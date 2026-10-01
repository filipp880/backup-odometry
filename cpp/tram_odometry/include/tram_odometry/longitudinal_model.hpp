// Longitudinal dynamics of the tram with adhesion limiting.
//
//   m_eff * dv/dt = F_drive - F_brake - F_roll - F_aero - F_grade - F_curve
//
// The drive force can never exceed what the wheel-rail contact can transmit,
// so the model also yields the *adhesion force ceiling* used by the slip
// detector: if the demanded wheel force exceeds it, odometry is wrong.
#pragma once

#include <algorithm>
#include <cmath>

#include "tram_odometry/params.hpp"

namespace tram {

class LongitudinalModel {
 public:
  LongitudinalModel(const DynamicsParams& dp, const VehicleParams& vp,
                    const AdhesionParams& ap);

  struct Input {
    double v = 0.0;
    double drive_force = 0.0;  ///< N, from the traction model
    double brake_force = 0.0;  ///< N, from the traction model
    double grade = 0.0;        ///< rad, path inclination
    double curvature = 0.0;    ///< 1/m
    double mu = 0.35;          ///< current adhesion estimate
    /// Learned residual on the acceleration, m/s^2 (MlCorrector). Zero when the
    /// corrector is disabled, so the pure-physics behaviour is bit-for-bit the
    /// pre-ML one.
    double residual_accel = 0.0;
  };

  /// Net acceleration, m/s^2 (adhesion limited, creep handled).
  double acceleration(const Input& in) const;

  /// Velocity after one step, with sign-consistent creep handling.
  double stepVelocity(const Input& in, double v, double dt) const;

  double resistance(double v, double grade, double curvature) const;
  double adhesionLimit(double mu) const;
  double effectiveMass() const { return m_eff_; }

  /// Slip ratio implied by a drive force that exceeds the adhesion limit.
  double slipFromExcess(double force_demand, double force_limit) const;

  /// Slow online adaptation of the rolling resistance from steady-state runs.
  void adaptRollingResistance(double v, double a_meas, double f_other, double dt);

  double c_rr() const { return c_rr_; }
  void setCRr(double v) { c_rr_ = std::clamp(v, 0.001, 0.05); }

 private:
  DynamicsParams dp_;
  VehicleParams vp_;
  AdhesionParams ap_;
  double m_eff_ = 0.0;
  double c_rr_ = 0.006;
  double v_steady_min_ = 1.0;  ///< below this the vehicle is considered stopped
};

}  // namespace tram
