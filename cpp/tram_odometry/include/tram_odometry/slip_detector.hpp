// Slip / anomaly detector.
//
// Four independent tests from the task statement are evaluated every cycle and
// fused into a single confidence in the wheel odometry, plus an estimate of the
// longitudinal slip ratio (a "diagnostic status of slip" for the jury).
#pragma once

#include "tram_odometry/params.hpp"
#include "tram_odometry/signal_filter.hpp"
#include "tram_odometry/types.hpp"

namespace tram {

struct SlipFeatures {
  double t = 0.0;
  double v_front = 0.0;   ///< m/s, from the front tachometers
  double v_rear = 0.0;    ///< m/s
  double a_wheel = 0.0;   ///< d(v_wheel)/dt, m/s^2
  double a_model = 0.0;   ///< model prediction, m/s^2
  double yaw_rate = 0.0;  ///< rad/s, 0 when unavailable
  double wheelbase = 7.55;  /**< расстояние между осями вращения тележек (per spec) */
  double drive_force = 0.0;  ///< N
  double brake_force = 0.0;  ///< N
  double adhesion_limit = 0.0;  ///< N
  /// Effective mass the model actually accelerates, kg. Required because
  /// mass_kg + rot_inertia_kgm2 adds kilograms to kilogram-metres squared.
  double effective_mass = 38000.0;
  bool front_valid = false;
  bool rear_valid = false;
  bool driver_valid = false;
  bool frozen_front = false;
  bool frozen_rear = false;
  bool dropout = false;
};

class SlipDetector {
 public:
  SlipDetector(const AdhesionParams& ap, const FilterParams& fp, const VehicleParams& vp);

  /// Returns the confidence in the wheel odometry, [0, 1].
  double update(const SlipFeatures& f, double dt);

  double trust() const { return trust_; }
  double slipIndex() const { return slip_index_; }
  double slipRatio() const { return slip_ratio_; }
  SlipReason reason() const { return reason_; }
  void reset();

 private:
  AdhesionParams ap_;
  FilterParams fp_;
  VehicleParams vp_;
  LowPassFilter trust_lp_;
  LowPassFilter slip_lp_;
  double trust_ = 0.5;
  double slip_index_ = 0.0;
  double slip_ratio_ = 0.0;
  double mu_ = 0.35;
  SlipReason reason_ = SlipReason::kNone;
};

}  // namespace tram
