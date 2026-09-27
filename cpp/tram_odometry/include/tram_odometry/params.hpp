// Configuration structures for tram_odometry.
// Values are loaded from config/params.yaml via rclcpp parameters; every field
// has a safe default so the node also runs with an empty config.
#pragma once

#include <string>
#include <vector>

namespace tram {

struct VehicleParams {
  std::string profile = "lion";
  double mass_kg = 38000.0;
  double wheel_radius_m = 0.30;
  int n_wheels_driven = 4;
  double wheelbase_m = 7.55;  /**< расстояние между осями вращения тележек вокруг вертикальной оси (per spec) */
  double front_overhang_m = 0.45;
  double rear_overhang_m = 0.45;
  double driveline_efficiency = 0.90;
  double gear_ratio = 1.0;
  double rot_inertia_kgm2 = 22.0;
  double g = 9.80665;
};

struct TractionParams {
  double u_min = -1.0;
  double u_max = 1.0;
  std::vector<double> torque_curve_u{0.0, 0.05, 0.15, 0.30, 0.50, 0.70, 0.85, 1.0};
  std::vector<double> torque_curve_tau{0.0, 0.35, 0.62, 0.80, 0.91, 0.97, 1.00, 1.00};
  double max_tractive_effort_n = 90000.0;
  double max_brake_effort_n = 120000.0;
  double rated_power_w = 300000.0;
  bool constant_power_limit = true;
  bool creep_compensation = true;
  double actuator_tau_s = 0.20;
  double brake_tau_s = 0.20;
};

struct DynamicsParams {
  double c_rr = 0.0060;
  double rho_air = 1.225;
  double cd_a = 12.0;
  double curve_resistance_coeff = 0.0015;
  std::string grade_source = "map";  // map | none
  double grade_max = 0.06;
};

struct AdhesionParams {
  // Steel wheel on steel rail: the usable adhesion coefficient is ~0.1, not the
  // 0.3-0.4 of a road tyre. At 0.35 the model could predict 3.4 m/s^2 for a
  // 38 t vehicle, which is far beyond what a tram can actually pull and beyond
  // the ~1 m/s^2 the contact patch can transmit.
  double mu_peak = 0.10;
  double mu_wet_factor = 0.65;
  double slip_peak = 0.12;
  double slip_hard = 0.45;
  double c_slip = 1.55;
  double b_slip = 8.0;
  double e_slip = 0.97;
  double bogie_mismatch_warn = 0.25;
  double bogie_mismatch_fault = 0.80;
};

struct FilterParams {
  int wheel_hampel_window = 7;
  double wheel_hampel_sigma = 3.0;
  double wheel_cutoff_hz = 25.0;
  double driver_cutoff_hz = 10.0;
  double gnss_cutoff_hz = 1.0;
  double velocity_cutoff_hz = 8.0;
  double accel_limit_mps2 = 1.6;
  double decel_limit_mps2 = 2.2;
  double freeze_timeout_s = 0.7;
  double dropout_tolerance_s = 0.35;
};

struct ObserverParams {
  std::string type = "eskf";  // ekf | eskf
  double sigma_v = 0.28;
  double sigma_a = 1.10;
  double sigma_scale = 0.010;
  double sigma_slip = 0.35;
  double r_wheel = 0.045;
  double r_wheel_degraded = 4.0;
  double r_gnss_pos = 0.80;
  double r_gnss_vel = 0.25;
  double gate_chi2 = 5.99;
  double gnss_max_age_s = 0.35;
  int gnss_min_sats = 6;
  double gnss_init_window_s = 2.5;
  std::string gnss_trust_after_init = "gated";  // gated | off
  double scale_adaptive_rate = 0.02;
  double slip_adaptive_rate = 0.05;
  double trust_min = 0.02;
  double trust_max = 0.98;
};

struct FrameParams {
  std::string mode = "mgrs_relative";  // mgrs_relative | mgrs_absolute | enu_local
  int utm_zone = 0;                    // 0 = autodetect from GNSS latitude
  std::string datum = "WGS84";
  double origin_lat = 0.0;
  double origin_lon = 0.0;
  double origin_alt = 0.0;
  bool xy_relative = true;  ///< subtract the origin easting/northing
  bool z_relative = false;  ///< keep absolute altitude (matches GNSS ellipsoidal height)
  bool publish_enu_twist = true;
};

/// GNSS is only trusted during the first seconds, and its quality is not
/// guaranteed, so every gate is explicit and conservative.
struct GnssParams {
  double baseline_m = 0.0;          ///< antenna baseline, 0 = not provided yet
  double baseline_heading_deg = 0.0;  ///< baseline direction in the vehicle frame
  bool use_aux_for_yaw = true;      ///< derive yaw rate from the antenna baseline
  double max_position_jump_m = 25.0;  ///< reject teleports
  double max_speed_mps = 35.0;        ///< reject impossible GNSS speeds
  double min_speed_for_calib_mps = 0.5;
  double min_travel_for_calib_m = 30.0;
  double innovation_gate_m = 30.0;    ///< reject GNSS disagreeing with dead reckoning
  double heading_rate_limit_rad_s = 0.5;
};


/// Learned residual corrector. See docs/04_ml_contract.md and ml_features.hpp.
struct MlParams {
  bool enable = false;             ///< без модели работает чистая физика
  std::string model_dir = "";      ///< каталог с дескриптором и весами
  std::string descriptor = "ml_model.yaml";
  bool strict = false;             ///< true = не падать, а требовать валидную модель
  double blend = 1.0;              ///< 0 = только физика, 1 = полная коррекция
  double max_a_residual = 1.50;    ///< м/с^2, потолок остаточной поправки
  double max_log_scale = 0.05;     ///< потолок коррекции масштаба одометрии
  double mu_min = 0.05;
  double mu_max = 0.90;
  bool use_mu_output = false;      ///< брать mu из модели вместо оценки наблюдателя
  int max_consecutive_failures = 20;  ///< после N отказов модель отключается
  double max_inference_ms = 2.0;   ///< бюджет на один вызов; превышение -> degraded
  double drop_when_untrusted = 0.0;  ///< если доверие к одометрии ниже, модель глушится
  bool dump_features = false;         ///< выгрузка 16 признаков в CSV для обучения
  std::string dump_path = "features_dump.csv";
};

struct PathMapParams {
  bool enable = true;
  std::string file;
  // Two directions of the same route, in the judge map frame. The direction
  // is picked from the sign of dx/dt once min_travel_m of odometry travel has
  // accumulated: the judge localisation is still 500 m off the route for the
  // first 220 s, so its own coordinates cannot be used to choose.
  std::string file_fwd;
  std::string file_rev;
  // Odometry travel before the map may constrain the position. Below that the
  // estimator runs blind rather than snapping to a map it cannot yet follow.
  double min_travel_m = 200.0;
  std::string frame_convention = "utm";
  double search_radius_m = 25.0;
  double max_projection_error_m = 8.0;
  bool publish_s = true;
};

struct TopicsParams {
  std::string front_bogie = "/vehicle/front_bogie_velocity";
  std::string rear_bogie = "/vehicle/rear_bogie_velocity";
  std::string driver_cmd = "/vehicle/driver_position_cmd";
  std::string gnss_fix = "/sensing/gnss/master/fix";
  std::string gnss_vel = "/sensing/gnss/master/vel";
  std::string gnss_fix_aux = "/sensing/gnss/rover/fix";
  std::string gnss_vel_aux = "/sensing/gnss/rover/vel";
  std::string out_velocity = "/result/velocity";
  std::string out_position = "/result/position";
  std::string out_diagnostics = "/result/diagnostics";
  std::string out_latency = "/result/latency";
};

struct RatesParams {
  double output_hz = 50.0;
  double watchdog_timeout_s = 0.5;
  double diagnostics_hz = 2.0;
};

struct RuntimeParams {
  bool realtime_hint = true;
  double max_execution_time_warn = 0.010;
  int history_len = 200;
  std::string log_level = "info";
};

struct Params {
  std::string node_name = "tram_odometry";
  std::string frame_id_map = "map";
  /// Frame used for the relative odometry fallback when no geodetic origin is
  /// available (bags without any GNSS topics).
  std::string frame_id_odom = "odom_local";
  bool use_sim_time = true;
  TopicsParams topics;
  RatesParams rates;
  FrameParams frame;
  GnssParams gnss;
  VehicleParams vehicle;
  TractionParams traction;
  DynamicsParams dynamics;
  AdhesionParams adhesion;
  FilterParams filters;
  ObserverParams observer;
  MlParams ml;
  PathMapParams path_map;
  RuntimeParams runtime;
};

}  // namespace tram

namespace rclcpp {
class Node;
}

namespace tram {
/// Declares every parameter and loads defaults/config. Must be called once.
Params declare_params(rclcpp::Node& node);
}  // namespace tram
