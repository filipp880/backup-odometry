#include "tram_odometry/params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace tram {
namespace {

template <typename T>
void load(rclcpp::Node& node, const std::string& name, T& out) {
  out = node.declare_parameter<T>(name, out);
}

}  // namespace

std::string resolve_asset(const std::string& rel, const std::string& share_subdir) {
  namespace fs = std::filesystem;
  if (rel.empty()) return {};
  const fs::path p(rel);
  if (p.is_absolute()) return fs::exists(p) ? p.string() : std::string{};
  try {
    const fs::path share =
        fs::path(ament_index_cpp::get_package_share_directory("tram_odometry"));
    const fs::path cand = share / share_subdir / rel;
    if (fs::exists(cand)) return cand.string();
  } catch (const std::exception&) {
    // Package not installed (running from a build tree). Fall through to CWD.
  }
  if (fs::exists(p)) return p.string();
  return {};
}

Params declare_params(rclcpp::Node& node) {
  Params p;

  // --- identity ------------------------------------------------------------
  load(node, "node_name", p.node_name);
  load(node, "frame_id_map", p.frame_id_map);
  load(node, "frame_id_odom", p.frame_id_odom);
  // use_sim_time is declared by rclcpp::Node itself, so it must be read, not
  // declared again: re-declaring it throws ParameterAlreadyDeclaredException.
  if (node.has_parameter("use_sim_time")) {
    p.use_sim_time = node.get_parameter("use_sim_time").as_bool();
  } else {
    node.declare_parameter<bool>("use_sim_time", p.use_sim_time);
  }

  // --- topics --------------------------------------------------------------
  load(node, "topics.front_bogie", p.topics.front_bogie);
  load(node, "topics.rear_bogie", p.topics.rear_bogie);
  load(node, "topics.driver_cmd", p.topics.driver_cmd);
  load(node, "topics.gnss_fix", p.topics.gnss_fix);
  load(node, "topics.gnss_vel", p.topics.gnss_vel);
  load(node, "topics.gnss_fix_aux", p.topics.gnss_fix_aux);
  load(node, "topics.gnss_vel_aux", p.topics.gnss_vel_aux);
  load(node, "topics.out_velocity", p.topics.out_velocity);
  load(node, "topics.out_position", p.topics.out_position);
  load(node, "topics.out_diagnostics", p.topics.out_diagnostics);
  load(node, "topics.out_latency", p.topics.out_latency);

  // --- rates ---------------------------------------------------------------
  load(node, "rates.output_hz", p.rates.output_hz);
  load(node, "rates.watchdog_timeout_s", p.rates.watchdog_timeout_s);
  load(node, "rates.diagnostics_hz", p.rates.diagnostics_hz);

  // --- frame ---------------------------------------------------------------
  load(node, "frame.mode", p.frame.mode);
  load(node, "frame.utm_zone", p.frame.utm_zone);
  load(node, "frame.datum", p.frame.datum);
  load(node, "frame.origin_lat", p.frame.origin_lat);
  load(node, "frame.origin_lon", p.frame.origin_lon);
  load(node, "frame.origin_alt", p.frame.origin_alt);
  load(node, "frame.xy_relative", p.frame.xy_relative);
  load(node, "frame.z_relative", p.frame.z_relative);
  load(node, "frame.publish_enu_twist", p.frame.publish_enu_twist);

  // --- gnss ----------------------------------------------------------------
  load(node, "gnss.baseline_m", p.gnss.baseline_m);
  load(node, "gnss.baseline_heading_deg", p.gnss.baseline_heading_deg);
  load(node, "gnss.use_aux_for_yaw", p.gnss.use_aux_for_yaw);
  load(node, "gnss.max_position_jump_m", p.gnss.max_position_jump_m);
  load(node, "gnss.max_speed_mps", p.gnss.max_speed_mps);
  load(node, "gnss.min_speed_for_calib_mps", p.gnss.min_speed_for_calib_mps);
  load(node, "gnss.min_travel_for_calib_m", p.gnss.min_travel_for_calib_m);
  load(node, "gnss.innovation_gate_m", p.gnss.innovation_gate_m);
  load(node, "gnss.heading_rate_limit_rad_s", p.gnss.heading_rate_limit_rad_s);

  // --- vehicle -------------------------------------------------------------
  // The profile is applied first so that its values become the defaults of the
  // individual parameters and can still be overridden from the YAML file.
  const std::string profile = node.declare_parameter<std::string>("vehicle.profile", p.vehicle.profile);
  p.vehicle.profile = profile;
  if (profile == "lion") {
    p.vehicle.mass_kg = 38000.0;
    p.vehicle.wheel_radius_m = 0.30;
  } else if (profile == "lion_empty") {
    p.vehicle.mass_kg = 30000.0;
    p.vehicle.wheel_radius_m = 0.30;
  } else if (profile == "custom") {
    // keep the struct defaults
  }
  load(node, "vehicle.mass_kg", p.vehicle.mass_kg);
  load(node, "vehicle.wheel_radius_m", p.vehicle.wheel_radius_m);
  load(node, "vehicle.n_wheels_driven", p.vehicle.n_wheels_driven);
  load(node, "vehicle.wheelbase_m", p.vehicle.wheelbase_m);
  load(node, "vehicle.front_overhang_m", p.vehicle.front_overhang_m);
  load(node, "vehicle.rear_overhang_m", p.vehicle.rear_overhang_m);
  load(node, "vehicle.driveline_efficiency", p.vehicle.driveline_efficiency);
  load(node, "vehicle.gear_ratio", p.vehicle.gear_ratio);
  load(node, "vehicle.rot_inertia_kgm2", p.vehicle.rot_inertia_kgm2);
  load(node, "vehicle.g", p.vehicle.g);

  // --- traction ------------------------------------------------------------
  load(node, "traction.u_min", p.traction.u_min);
  load(node, "traction.u_max", p.traction.u_max);
  load(node, "traction.torque_curve_u", p.traction.torque_curve_u);
  load(node, "traction.torque_curve_tau", p.traction.torque_curve_tau);
  load(node, "traction.max_tractive_effort_n", p.traction.max_tractive_effort_n);
  load(node, "traction.max_brake_effort_n", p.traction.max_brake_effort_n);
  load(node, "traction.rated_power_w", p.traction.rated_power_w);
  load(node, "traction.constant_power_limit", p.traction.constant_power_limit);
  load(node, "traction.creep_compensation", p.traction.creep_compensation);
  load(node, "traction.actuator_tau_s", p.traction.actuator_tau_s);
  load(node, "traction.brake_tau_s", p.traction.brake_tau_s);

  // --- dynamics ------------------------------------------------------------
  load(node, "dynamics.c_rr", p.dynamics.c_rr);
  load(node, "dynamics.rho_air", p.dynamics.rho_air);
  load(node, "dynamics.cd_a", p.dynamics.cd_a);
  load(node, "dynamics.curve_resistance_coeff", p.dynamics.curve_resistance_coeff);
  load(node, "dynamics.grade_source", p.dynamics.grade_source);
  load(node, "dynamics.grade_max", p.dynamics.grade_max);

  // --- adhesion ------------------------------------------------------------
  load(node, "adhesion.mu_peak", p.adhesion.mu_peak);
  load(node, "adhesion.mu_wet_factor", p.adhesion.mu_wet_factor);
  load(node, "adhesion.slip_peak", p.adhesion.slip_peak);
  load(node, "adhesion.slip_hard", p.adhesion.slip_hard);
  load(node, "adhesion.c_slip", p.adhesion.c_slip);
  load(node, "adhesion.b_slip", p.adhesion.b_slip);
  load(node, "adhesion.e_slip", p.adhesion.e_slip);
  load(node, "adhesion.bogie_mismatch_warn", p.adhesion.bogie_mismatch_warn);
  load(node, "adhesion.bogie_mismatch_fault", p.adhesion.bogie_mismatch_fault);

  // --- filters -------------------------------------------------------------
  load(node, "filters.wheel_hampel_window", p.filters.wheel_hampel_window);
  load(node, "filters.wheel_hampel_sigma", p.filters.wheel_hampel_sigma);
  load(node, "filters.wheel_cutoff_hz", p.filters.wheel_cutoff_hz);
  load(node, "filters.driver_cutoff_hz", p.filters.driver_cutoff_hz);
  load(node, "filters.gnss_cutoff_hz", p.filters.gnss_cutoff_hz);
  load(node, "filters.velocity_cutoff_hz", p.filters.velocity_cutoff_hz);
  load(node, "filters.accel_limit_mps2", p.filters.accel_limit_mps2);
  load(node, "filters.decel_limit_mps2", p.filters.decel_limit_mps2);
  load(node, "filters.freeze_timeout_s", p.filters.freeze_timeout_s);
  load(node, "filters.dropout_tolerance_s", p.filters.dropout_tolerance_s);

  // --- observer ------------------------------------------------------------
  load(node, "observer.type", p.observer.type);
  load(node, "observer.sigma_v", p.observer.sigma_v);
  load(node, "observer.sigma_a", p.observer.sigma_a);
  load(node, "observer.sigma_scale", p.observer.sigma_scale);
  load(node, "observer.sigma_slip", p.observer.sigma_slip);
  load(node, "observer.r_wheel", p.observer.r_wheel);
  load(node, "observer.r_wheel_degraded", p.observer.r_wheel_degraded);
  load(node, "observer.r_gnss_pos", p.observer.r_gnss_pos);
  load(node, "observer.r_gnss_vel", p.observer.r_gnss_vel);
  load(node, "observer.gate_chi2", p.observer.gate_chi2);
  load(node, "observer.gnss_max_age_s", p.observer.gnss_max_age_s);
  load(node, "observer.gnss_min_sats", p.observer.gnss_min_sats);
  load(node, "observer.gnss_init_window_s", p.observer.gnss_init_window_s);
  load(node, "observer.gnss_trust_after_init", p.observer.gnss_trust_after_init);
  load(node, "observer.scale_adaptive_rate", p.observer.scale_adaptive_rate);
  load(node, "observer.slip_adaptive_rate", p.observer.slip_adaptive_rate);
  load(node, "observer.trust_min", p.observer.trust_min);
  load(node, "observer.trust_max", p.observer.trust_max);
  load(node, "observer.q_heading_rad2_s", p.observer.q_heading_rad2_s);
  load(node, "observer.cov_heading_floor_rad2", p.observer.cov_heading_floor_rad2);
  load(node, "observer.cov_heading_max_rad2", p.observer.cov_heading_max_rad2);

  // --- learned residual corrector -----------------------------------------
  load(node, "ml.enable", p.ml.enable);
  load(node, "ml.model_dir", p.ml.model_dir);
  load(node, "ml.descriptor", p.ml.descriptor);
  load(node, "ml.strict", p.ml.strict);
  load(node, "ml.blend", p.ml.blend);
  load(node, "ml.max_a_residual", p.ml.max_a_residual);
  load(node, "ml.max_log_scale", p.ml.max_log_scale);
  load(node, "ml.mu_min", p.ml.mu_min);
  load(node, "ml.mu_max", p.ml.mu_max);
  load(node, "ml.use_mu_output", p.ml.use_mu_output);
  load(node, "ml.max_consecutive_failures", p.ml.max_consecutive_failures);
  load(node, "ml.max_inference_ms", p.ml.max_inference_ms);
  load(node, "ml.drop_when_untrusted", p.ml.drop_when_untrusted);
  load(node, "ml.dump_features", p.ml.dump_features);
  load(node, "ml.dump_path", p.ml.dump_path);

  // --- path map ------------------------------------------------------------
  load(node, "path_map.enable", p.path_map.enable);
  load(node, "path_map.file", p.path_map.file);
  load(node, "path_map.file_fwd", p.path_map.file_fwd);
  load(node, "path_map.file_rev", p.path_map.file_rev);
  load(node, "path_map.min_travel_m", p.path_map.min_travel_m);
  load(node, "path_map.frame_offset_e", p.path_map.frame_offset_e);
  load(node, "path_map.frame_offset_n", p.path_map.frame_offset_n);
  load(node, "path_map.search_radius_m", p.path_map.search_radius_m);
  load(node, "path_map.max_projection_error_m", p.path_map.max_projection_error_m);
  load(node, "path_map.publish_s", p.path_map.publish_s);

  // --- runtime -------------------------------------------------------------
  load(node, "runtime.realtime_hint", p.runtime.realtime_hint);
  load(node, "runtime.max_execution_time_warn", p.runtime.max_execution_time_warn);
  load(node, "runtime.history_len", p.runtime.history_len);
  load(node, "runtime.log_level", p.runtime.log_level);

  return p;
}

}  // namespace tram
