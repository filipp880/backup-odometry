#include "tram_odometry/traction_model.hpp"

#include <algorithm>
#include <cmath>

namespace tram {

TractionModel::TractionModel(const TractionParams& tp)
    : tp_(tp),
      drive_lag_(tp.actuator_tau_s),
      brake_lag_(tp.brake_tau_s) {
  // Rebuild the lags if the params object was re-tuned.
  drive_lag_ = FirstOrderLag(tp_.actuator_tau_s);
  brake_lag_ = FirstOrderLag(tp_.brake_tau_s);
  reset();
}

void TractionModel::reset() {
  drive_lag_.reset(0.0);
  brake_lag_.reset(0.0);
  drive_force_ = 0.0;
  brake_force_ = 0.0;
}

double TractionModel::shapeFactor(double u) const {
  const std::vector<double>& xs = tp_.torque_curve_u;
  const std::vector<double>& ys = tp_.torque_curve_tau;
  const size_t n = std::min(xs.size(), ys.size());
  if (n == 0) return 0.0;
  if (n == 1) return std::clamp(ys[0], 0.0, 1.0);

  const double x = std::clamp(u, 0.0, 1.0);
  if (x <= xs.front()) return ys.front();
  if (x >= xs.back()) return ys.back();
  for (size_t i = 1; i < n; ++i) {
    if (x <= xs[i]) {
      const double dx = xs[i] - xs[i - 1];
      if (dx <= 1e-9) return ys[i];
      const double w = (x - xs[i - 1]) / dx;
      return ys[i - 1] + w * (ys[i] - ys[i - 1]);
    }
  }
  return ys.back();
}

double TractionModel::powerLimit(double v) const {
  if (!tp_.constant_power_limit) return 1.0;
  const double v_eps = 0.5;
  const double vv = std::max(std::fabs(v), v_eps);
  if (tp_.rated_power_w <= 0.0) return 1.0;
  return std::min(1.0, tp_.rated_power_w / (tp_.max_tractive_effort_n * vv));
}

double TractionModel::step(double u, double v, double dt) {
  const double u_clamped = std::clamp(u, tp_.u_min, tp_.u_max);

  if (u_clamped >= 0.0) {
    const double shape = shapeFactor(u_clamped);
    const double f_cmd = shape * tp_.max_tractive_effort_n * powerLimit(v);
    const double f = drive_lag_.step(f_cmd, dt);
    drive_force_ = std::max(0.0, f);
    brake_force_ = 0.0;
  } else {
    const double shape = shapeFactor(-u_clamped);
    const double f_cmd = shape * tp_.max_brake_effort_n;
    const double f = brake_lag_.step(f_cmd, dt);
    brake_force_ = std::max(0.0, f);
    drive_force_ = 0.0;
  }
  return drive_force_ - brake_force_;
}

}  // namespace tram
