#include "tram_odometry/ml_corrector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace tram {

MlCorrector::MlCorrector() = default;
MlCorrector::~MlCorrector() = default;

void MlCorrector::reset() {
  last_ = CorrectorOutput{};
  consecutive_failures_ = 0;
  last_u_ = 0.0;
  last_t_ = -1e9;
  if (has_model_ && !degraded_) {
    status_ = "ready";
  }
}

void MlCorrector::disable(const char* reason) {
  degraded_ = true;
  has_model_ = false;
  status_ = reason;
  stats_.degraded = true;
  stats_.backend = backend_ ? backend_->name() : "null";
}

void MlCorrector::configure(const MlParams& params, const std::string& model_dir) {
  p_ = params;
  norm_ = FeatureNorm::identity();
  desc_ = ModelDescriptor{};
  backend_.reset();
  has_model_ = false;
  degraded_ = false;
  consecutive_failures_ = 0;
  stats_ = InferenceStats{};
  last_ = CorrectorOutput{};

  if (!p_.enable) {
    backend_ = std::make_unique<NullBackend>();
    stats_.backend = "null";
    status_ = "disabled by config (ml.enable=false)";
    return;
  }

  // The `model_dir` argument is the one the caller already resolved - absolute,
  // then the installed share directory, then the working directory. It must win
  // over p_.model_dir, which is the raw parameter and is relative ("models" in
  // config/params.yaml). Preferring the parameter threw the resolved path away
  // and made the corrector open the relative "models/ml_model.yaml", so from an
  // install tree it reported "descriptor rejected: cannot open descriptor" while
  // the artifact was sitting in the share directory all along.
  std::string dir = !model_dir.empty() ? model_dir : p_.model_dir;
  if (dir.empty()) {
    backend_ = std::make_unique<NullBackend>();
    stats_.backend = "null";
    status_ = "no model_dir configured";
    return;
  }

  const std::string path = dir + "/" + p_.descriptor;
  desc_ = load_descriptor(path);
  if (!desc_.valid) {
    backend_ = std::make_unique<NullBackend>();
    stats_.backend = "null";
    status_ = "descriptor rejected: " + desc_.error;
    if (p_.strict) {
      disable("strict mode: model artifact is invalid");
    }
    return;
  }

  norm_ = desc_.norm();
  backend_ = make_backend(desc_, dir);
  // NullBackend is the "no usable artifact" sentinel: it is itself ready (it
  // runs and returns zeros), so readiness alone cannot distinguish it from a
  // real backend. Fall back to pure physics in that case.
  const bool backend_is_null = !backend_ || std::string(backend_->name()) == "null";
  if (backend_is_null || !backend_->ready()) {
    std::string why = backend_ ? backend_->lastError() : std::string("null backend");
    if (why.empty()) why = "weight file missing or unreadable";
    backend_ = std::make_unique<NullBackend>();
    status_ = "backend unavailable (" + std::string(backend_->name()) + "): " + why;
    stats_.backend = backend_->name();
    if (p_.strict) disable("strict mode: backend unavailable");
    return;
  }

  stats_.backend = backend_->name();
  stats_.model_id = desc_.model_id;
  has_model_ = true;
  degraded_ = false;
  status_ = std::string("ready: ") + backend_->name() + " / " + desc_.model_id;
}

void MlCorrector::buildFeatures(const CorrectorInput& in, FeatureVector* out) {
  if (!out) return;
  const double force = std::fabs(in.drive_force - in.brake_force);
  const double limit = std::max(1.0, std::fabs(in.adhesion_limit));
  (*out)[kFControllerPos] = in.u;
  (*out)[kFSpeed] = in.v;
  (*out)[kFAccelModel] = in.a_model;
  (*out)[kFGrade] = in.grade;
  (*out)[kFMu] = in.mu;
  (*out)[kFOmegaFront] = in.omega_front;
  (*out)[kFOmegaRear] = in.omega_rear;
  (*out)[kFOdomScale] = in.b_scale;
  (*out)[kFSlipIndex] = in.slip_index;
  (*out)[kFTrust] = in.trust;
  (*out)[kFCmdRate] = in.cmd_rate;
  (*out)[kFDt] = in.dt;
  (*out)[kFAbsU] = std::fabs(in.u);
  (*out)[kFUSq] = in.u * in.u;
  (*out)[kFVSq] = in.v * in.v;
  (*out)[kFForceRatio] = std::clamp(force / limit, 0.0, 1.0);
}

CorrectorOutput MlCorrector::step(const CorrectorInput& in) {
  CorrectorOutput out;
  if (!has_model_ || !backend_ || !backend_->ready()) {
    out.degraded = degraded_ || (p_.enable && !has_model_);
    return out;
  }
  if (p_.blend <= 0.0) return out;

  // Non-finite state must never reach the network: a single NaN would poison
  // the correction and, through the integrator, the whole trajectory.
  if (!std::isfinite(in.u) || !std::isfinite(in.v) || !std::isfinite(in.a_model)) {
    stats_.rejected++;
    out.degraded = true;
    return out;
  }
  if (p_.drop_when_untrusted > 0.0 && in.trust < p_.drop_when_untrusted) {
    // Heavily slipped odometry: the network was trained on model-dominated
    // states, so its output is not trustworthy here either.
    return out;
  }

  buildFeatures(in, &raw_);
  if (!raw_.finite()) {
    stats_.rejected++;
    out.degraded = true;
    return out;
  }
  norm_.apply(raw_, scaled_);

  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = backend_->run(scaled_.x, kNumFeatures, out_, kNumOutputs);
  const auto t1 = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  stats_.calls++;
  stats_.mean_ms += (ms - stats_.mean_ms) / static_cast<double>(stats_.calls);
  stats_.max_ms = std::max(stats_.max_ms, ms);
  out.inference_ms = ms;

  if (!ok) {
    stats_.failures++;
    if (++consecutive_failures_ >= std::max(1, p_.max_consecutive_failures)) {
      disable("backend failed repeatedly, disabled");
    }
    out.degraded = degraded_;
    return out;
  }
  consecutive_failures_ = 0;

  if (ms > p_.max_inference_ms && p_.max_inference_ms > 0.0) {
    stats_.overruns++;
    out.degraded = true;
    return out;
  }

  for (int i = 0; i < kNumOutputs; ++i) {
    if (!std::isfinite(out_[i])) {
      stats_.rejected++;
      out.degraded = true;
      return out;
    }
  }

  const double blend = std::clamp(p_.blend, 0.0, 1.0);
  const double a_raw = out_[kOAresidual] * blend;
  const double s_raw = out_[kOLogScale] * blend;
  const double a_lim = std::min(std::abs(p_.max_a_residual), desc_.limits.max_a_residual);
  const double s_lim = std::min(std::abs(p_.max_log_scale), desc_.limits.max_log_scale);

  out.a_residual = clampOutput(a_raw, -a_lim, a_lim);
  out.log_scale = clampOutput(s_raw, -s_lim, s_lim);
  out.mu_model = clampOutput(out_[kOMu], p_.mu_min, p_.mu_max);
  out.applied = true;

  if (std::fabs(a_raw) > a_lim + 1e-12 || std::fabs(s_raw) > s_lim + 1e-12) {
    stats_.rejected++;
  }

  last_ = out;
  last_u_ = in.u;
  last_t_ = in.t;
  return out;
}

}  // namespace tram
