// Non-linear traction/brake model: controller position -> wheel force.
//
//   F_wheel(u) = F_max * Phi(u) * P_lim(v)     limited by constant power
//   dF/dt      = (F_cmd - F) / tau
//
// Phi(u) is a piecewise-linear table (the real notch characteristic is neither
// linear in the handle position nor constant in speed). The limits live in
// TractionParams and are already expressed as wheel force in newtons, so the
// model needs no vehicle parameters: gear ratio and wheel radius belong to the
// effective-mass computation in LongitudinalModel, not here.
#pragma once

#include "tram_odometry/params.hpp"
#include "tram_odometry/signal_filter.hpp"

namespace tram {

class TractionModel {
 public:
  explicit TractionModel(const TractionParams& tp);

  void reset();

  /// Advances the drive train. u is the normalised controller position,
  /// v the current body speed in m/s. Returns the commanded wheel force (N)
  /// before adhesion limiting.
  double step(double u, double v, double dt);

  /// Normalised non-linear shape of the controller characteristic, [0, 1].
  double shapeFactor(double u) const;

  double driveForce() const { return drive_force_; }
  double brakeForce() const { return brake_force_; }
  double powerLimit(double v) const;

 private:
  TractionParams tp_;
  FirstOrderLag drive_lag_;
  FirstOrderLag brake_lag_;
  double drive_force_ = 0.0;
  double brake_force_ = 0.0;
};

}  // namespace tram
