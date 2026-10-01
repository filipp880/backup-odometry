#include "tram_odometry/longitudinal_model.hpp"

#include <algorithm>
#include <cmath>

namespace tram {

LongitudinalModel::LongitudinalModel(const DynamicsParams& dp, const VehicleParams& vp,
                                     const AdhesionParams& ap)
    : dp_(dp), vp_(vp), ap_(ap) {
  // Inertia of the rotating masses referred to the contact patch.
  const double r = std::max(1e-3, vp_.wheel_radius_m);
  const double ig2 = vp_.gear_ratio * vp_.gear_ratio;
  const double j_referred = vp_.rot_inertia_kgm2 * ig2 / (r * r);
  m_eff_ = vp_.mass_kg + j_referred;
  c_rr_ = dp_.c_rr;
}

double LongitudinalModel::resistance(double v, double grade, double curvature) const {
  const double g = vp_.g;
  const double m = vp_.mass_kg;
  const double f_roll = c_rr_ * m * g * std::cos(grade);
  const double f_aero = 0.5 * dp_.rho_air * dp_.cd_a * v * std::abs(v);
  const double f_grade = m * g * std::sin(grade);
  const double f_curve = dp_.curve_resistance_coeff * m * g * std::fabs(curvature);
  return f_roll + f_aero + f_grade + f_curve;
}

double LongitudinalModel::adhesionLimit(double mu) const {
  const double fz = vp_.mass_kg * vp_.g / 4.0 * std::max(1, vp_.n_wheels_driven);
  return std::max(0.0, mu) * fz;
}

double LongitudinalModel::slipFromExcess(double force_demand, double force_limit) const {
  if (force_limit <= 1e-6) return ap_.slip_hard;
  const double excess = force_demand / force_limit - 1.0;
  if (excess <= 0.0) return 0.0;
  // Linear interpolation between the peak and the fully sliding slip ratio.
  const double span = std::max(1e-3, ap_.slip_hard - ap_.slip_peak);
  return std::min(ap_.slip_hard, ap_.slip_peak + excess * span);
}

double LongitudinalModel::acceleration(const Input& in) const {
  const double mu = std::clamp(in.mu, 0.02, 1.5);
  const double f_limit = adhesionLimit(mu);

  // Limit the NET force, not each side separately. Clamping f_drive and
  // f_brake first and then subtracting the resistances leaves the result below
  // the ceiling, and an absurd demand would no longer saturate at the adhesion
  // limit. One clamp on the sum is both simpler and the physically correct
  // statement: the contact patch cannot transmit more than mu*m*g in either
  // direction.
  const double f_res = resistance(in.v, in.grade, in.curvature);
  double f_net = in.drive_force - in.brake_force - f_res;
  f_net = std::clamp(f_net, -f_limit, f_limit);
  const double a_phys = f_net / m_eff_;

  // The learned part only ever *adds* to the physical acceleration, and it is
  // already clipped upstream, so an absurd model cannot flip the sign of the
  // traction or brake decision taken by the caller.
  if (!std::isfinite(in.residual_accel)) return a_phys;
  return a_phys + in.residual_accel;
}

double LongitudinalModel::stepVelocity(const Input& in, double v, double dt) const {
  double a = acceleration(in);
  double v_new = v + a * dt;

  // A sign change of the velocity. Comparing the product v * v_new misses the
  // standstill case: at v == 0 the product is exactly 0, so 0 < 0 is false and
  // the vehicle is allowed to creep backwards off a standstill. The explicit
  // comparisons also catch "leaving zero in the wrong direction".
  const bool flips = (v > 0.0 && v_new < 0.0) || (v < 0.0 && v_new > 0.0) ||
                     (std::fabs(v) <= 1e-12 && v_new < 0.0);

  // Creep handling: rolling/braking friction must not push the vehicle
  // backwards. If the sign of the velocity flips and the controller commands
  // no traction, the tram is standing still.
  const bool braking = in.brake_force > 1.0;
  const bool coasting = in.drive_force <= 1.0 && !braking;
  const bool commanding = in.drive_force > 1.0;
  if (braking && flips && !commanding) {
    return 0.0;
  }
  if (coasting && std::fabs(v) < v_steady_min_ && std::fabs(a) * m_eff_ < 0.02 * vp_.mass_kg * vp_.g) {
    return 0.0;
  }
  if (commanding && flips && in.drive_force > 0.0) {
    // Braking harder than the traction available: allow the reversal only when
    // the controller really commands drive, otherwise clamp to zero.
    if (in.brake_force > 0.0) return 0.0;
  }
  return v_new;
}

void LongitudinalModel::adaptRollingResistance(double v, double a_meas, double f_other,
                                               double dt) {
  // Steady-state identification: c_rr such that the measured acceleration is
  // reproduced. Only trusted in a narrow, clearly rolling band.
  if (v < 1.0 || v > 20.0) return;
  if (dt <= 0.0) return;
  const double g = vp_.g;
  const double m = vp_.mass_kg;
  const double f_needed = a_meas * m_eff_ + f_other;
  const double c_new = f_needed / (m * g);
  if (!(c_new > 0.0) || c_new > 0.05) return;
  const double rate = std::min(0.05, dt * 0.05);
  c_rr_ += rate * (c_new - c_rr_);
  c_rr_ = std::clamp(c_rr_, 0.001, 0.05);
}

}  // namespace tram
