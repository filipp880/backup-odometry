// Plain-old-data types exchanged between the pipeline stages.
#pragma once

#include <cstdint>

namespace tram {

/// Wheel + controller measurement bundle aligned in time by the preprocessor.
struct InputSample {
  double t = 0.0;
  double omega_front = 0.0;  ///< rad/s
  double omega_rear = 0.0;   ///< rad/s
  double u = 0.0;            ///< normalised controller position, [-1, 1]
  bool front_valid = false;
  bool rear_valid = false;
  bool driver_valid = false;
};

/// GNSS sample (may be garbage: quality is not guaranteed by the task).
struct GnssSample {
  double t = 0.0;
  double lat = 0.0;  ///< deg
  double lon = 0.0;  ///< deg
  double alt = 0.0;  ///< m
  double ve = 0.0;   ///< m/s east
  double vn = 0.0;   ///< m/s north
  bool has_fix = false;
  bool has_vel = false;
  int sats = 0;
  uint8_t status = 0;  ///< NavSatStatus.STATUS_*
  double position_cov = -1.0;
};

/// Reason codes for the diagnostics topic.
enum class SlipReason : uint8_t {
  kNone = 0,
  kBogieMismatch,
  kYawMismatch,
  kTorqueAccelConflict,
  kFrozenSensor,
  kDropout,
  kAccelBeyondAdhesion,
  kOutlier,
};
const char* to_string(SlipReason r);

/// Output of the whole pipeline: what gets published.
struct Estimate {
  double t = 0.0;

  // --- core state ---
  double v = 0.0;         ///< longitudinal speed of the body, m/s
  double a = 0.0;         ///< acceleration, m/s^2
  double s = 0.0;         ///< travelled distance along the path, m
  double b_scale = 1.0;   ///< odometry scale (effective/nominal wheel radius)
  double kappa_front = 0.0;  ///< longitudinal slip ratio, front bogie
  double kappa_rear = 0.0;   ///< longitudinal slip ratio, rear bogie
  double mu = 0.35;         ///< estimated adhesion coefficient
  double grade = 0.0;       ///< path grade used by the model

  // --- quality / diagnostics ---
  double trust = 0.5;      ///< confidence in wheel odometry, [0, 1]
  double slip_index = 0.0; ///< 0 = no slip, 1 = full slip
  double innovation = 0.0; ///< normalised wheel innovation (NIS-like)
  bool model_only = false; ///< true when odometry is not trusted at all
  bool gnss_used = false;  ///< GNSS accepted in this cycle
  bool initialized = false;
  SlipReason reason = SlipReason::kNone;  ///< dominant slip/failure reason

  // --- position ---
  double x = 0.0;     ///< MGRS/UTM easting (or ENU east) per frame config
  double y = 0.0;     ///< MGRS/UTM northing
  double z = 0.0;     ///< altitude
  double along = 0.0; ///< along-track coordinate on pathgraph, m
  double cross = 0.0; ///< cross-track error to pathgraph, m
  double heading = 0.0;  ///< path heading, rad
  double v_lat = 0.0;    ///< lateral speed (from GNSS baseline / map), m/s
  double yaw_rate = 0.0; ///< rad/s, from GNSS baseline when available

  // --- telemetry ---
  /// Wall-clock cost of the control cycle, which is the input-to-publish latency
  /// the case bounds at 100 ms. Previously this field held the age of the newest
  /// input instead, which with 10 Hz wheel topics cycles between 0 and 100 ms by
  /// construction and made a healthy node look like a 100 ms violation.
  double latency_ms = 0.0;
  double input_age_ms = 0.0;  ///< sim time since the newest /vehicle/* message
  double cycle_ms = 0.0;      ///< node cycle duration
  double wheel_force = 0.0; ///< modelled traction force at wheel, N
  double brake_force = 0.0; ///< modelled brake force at wheel, N

  // --- learned residual corrector (see docs/04_ml_contract.md) ---
  double ml_a_residual = 0.0;  ///< m/s^2 actually applied on top of physics
  double ml_log_scale = 0.0;   ///< log-domain odometry scale correction applied
  double ml_inference_ms = 0.0;
  bool ml_applied = false;     ///< false => this cycle ran on physics only
  bool ml_degraded = false;    ///< model missing, disabled or failing

  // --- covariances for nav_msgs/Odometry ---
  double cov_v = 0.0;
  double cov_a = 0.0;
  double cov_s = 0.0;
  double cov_heading = 0.0;
};

}  // namespace tram
