#include "tram_odometry/signal_filter.hpp"

#include <algorithm>
#include <cmath>

namespace tram {

// ---------------------------------------------------------------- HampelFilter

HampelFilter::HampelFilter(int window, double sigma)
    : window_(window > 1 ? (window > kMax ? kMax : window) : 1), sigma_(sigma) {
  reset();
}

void HampelFilter::reset() {
  for (int i = 0; i < kMax; ++i) buf_[i] = 0.0;
  count_ = 0;
  head_ = 0;
  median_ = 0.0;
  last_ = 0.0;
  has_value_ = false;
}

bool HampelFilter::push(double x) {
  if (!std::isfinite(x)) return true;  // NaN/inf are always outliers

  if (count_ < window_) {
    buf_[head_] = x;
    head_ = (head_ + 1) % window_;
    ++count_;
  } else {
    buf_[head_] = x;
    head_ = (head_ + 1) % window_;
  }

  for (int i = 0; i < count_; ++i) scratch_[i] = buf_[i];
  std::sort(scratch_, scratch_ + count_);
  const double med = (count_ % 2 == 1) ? scratch_[count_ / 2]
                                       : 0.5 * (scratch_[count_ / 2 - 1] + scratch_[count_ / 2]);
  median_ = med;

  // Absolute deviation around the median -> scaled MAD.
  for (int i = 0; i < count_; ++i) scratch_[i] = std::fabs(buf_[i] - med);
  std::sort(scratch_, scratch_ + count_);
  const double mad = (count_ % 2 == 1) ? scratch_[count_ / 2]
                                       : 0.5 * (scratch_[count_ / 2 - 1] + scratch_[count_ / 2]);
  const double scale = 1.4826 * mad;

  bool outlier = false;
  if (count_ >= 3) {
    if (scale > 1e-6) {
      outlier = std::fabs(x - med) > sigma_ * scale;
    } else {
      // Perfectly constant window: accept only values close to it.
      outlier = std::fabs(x - med) > 1e-3;
    }
  }
  if (!outlier) {
    last_ = x;
    has_value_ = true;
  }
  return outlier;
}

// --------------------------------------------------------------- LowPassFilter

LowPassFilter::LowPassFilter(double cutoff_hz, double nominal_dt) : cutoff_(cutoff_hz) {
  nominal_dt_ = nominal_dt > 1e-4 ? nominal_dt : 0.02;
  design(cutoff_, nominal_dt_);
}

void LowPassFilter::reset() {
  s1_.reset();
  s2_.reset();
}

void LowPassFilter::design(double cutoff_hz, double dt) {
  const double fs = 1.0 / dt;
  double fc = cutoff_hz;
  const double nyq = 0.5 * fs;
  if (fc >= nyq) fc = 0.9 * nyq;
  if (fc < 1e-3) fc = 1e-3;

  // Two cascaded single-pole sections instead of two 2nd-order Butterworth
  // biquads. A 4th-order Butterworth rings on a step (roughly 10 % overshoot for
  // a cascade of biquads), and a speed filter that overshoots feeds a false
  // acceleration straight into the observer. Cascading two first-order sections
  // keeps the rolloff (about -12 dB/oct) and a unity DC gain, but the step
  // response is a product of two monotone terms, so it can never overshoot.
  // K = pi * fc * dt is the bilinear-transformed pole of one RC section.
  const double K = M_PI * fc * dt;
  const double norm = 1.0 / (1.0 + K);

  Biquad q;
  q.b0 = K * norm;
  q.b1 = K * norm;
  q.b2 = 0.0;
  q.a1 = (K - 1.0) * norm;
  q.a2 = 0.0;
  s1_ = q;
  s2_ = q;
  designed_dt_ = dt;
  designed_cutoff_ = cutoff_hz;
}

double LowPassFilter::process(double x, double dt) {
  if (dt < 1e-5) dt = nominal_dt_;
  if (std::fabs(dt - designed_dt_) > 0.25 * designed_dt_ ||
      std::fabs(cutoff_ - designed_cutoff_) > 1e-6) {
    design(cutoff_, dt);
  }
  return s2_.step(s1_.step(x));
}

}  // namespace tram
