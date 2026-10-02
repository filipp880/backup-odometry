#include "tram_odometry/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>

namespace tram {
namespace {
// Window during which /result/position follows the GNSS fix directly.
constexpr double kGnssPublishWindowS = 2.5;
}  // namespace
namespace {
// One-shot wheel-scale calibration. 1.0 m/s and 0.2 m/s^2 are the same gates
// calibrateScaleFromVelocity and adaptFriction already use; the window is 5 s.
constexpr double kScaleCalMinSpeed = 1.0;
constexpr double kScaleCalMaxAccel = 0.2;
constexpr size_t kScaleCalWindow = 250;
}  // namespace
namespace {

constexpr double kKmhToMs = 1.0 / 3.6;
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

inline double wrap_pi(double a) {
  while (a > M_PI) a -= kTwoPi;
  while (a < -M_PI) a += kTwoPi;
  return a;
}

inline bool sane_latlon(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) && lat >= -90.0 && lat <= 90.0 &&
         lon >= -180.0 && lon <= 180.0;
}

}  // namespace

Estimator::Estimator(const Params& params, const std::string& ml_model_dir)
    : p_(params),
      traction_(params.traction),
      dynamics_(params.dynamics, params.vehicle, params.adhesion),
      slip_(params.adhesion, params.filters, params.vehicle),
      observer_(params.observer, params.vehicle) {
  const double dt_nom = 1.0 / std::max(1.0, p_.rates.output_hz);
  front_.hampel = HampelFilter(p_.filters.wheel_hampel_window, p_.filters.wheel_hampel_sigma);
  rear_.hampel = front_.hampel;
  front_.lpf = LowPassFilter(p_.filters.wheel_cutoff_hz, dt_nom);
  rear_.lpf = front_.lpf;
  front_.freeze = FreezeDetector(p_.filters.freeze_timeout_s);
  rear_.freeze = front_.freeze;
  driver_hampel_ = HampelFilter(p_.filters.wheel_hampel_window, 4.0);
  driver_lpf_ = LowPassFilter(p_.filters.driver_cutoff_hz, dt_nom);

  zone_ = p_.frame.utm_zone;

  if (p_.ml.dump_features) {
    // Header order is kFeatureNames from ml_features.hpp, verbatim, followed by
    // the regression target. ml_features_ is filled by the same call the network
    // sees, so the dumped values are exactly the ones used at inference time.
    const std::string path = p_.ml.dump_path.empty() ? "features_dump.csv" : p_.ml.dump_path;
    feature_dump_.open(path);
    if (feature_dump_.is_open()) {
      feature_dump_ << "t";
      for (int i = 0; i < kNumFeatures; ++i) feature_dump_ << "," << kFeatureNames[i];
      feature_dump_ << ",target_a_residual,target_log_scale,target_mu"
                       ",wheels_valid,label_from_gnss\n";
      feature_dump_.setf(std::ios::fixed);
      feature_dump_.precision(9);
    }
  }

  if (p_.frame.origin_lat != 0.0 || p_.frame.origin_lon != 0.0) {
    origin_lat_ = p_.frame.origin_lat;
    origin_lon_ = p_.frame.origin_lon;
    origin_alt_ = p_.frame.origin_alt;
    origin_utm_ = wgs84_to_utm(origin_lat_, origin_lon_, zone_);
    if (zone_ == 0 && origin_utm_.valid) zone_ = origin_utm_.zone;
    has_origin_ = origin_utm_.valid;
    origin_from_config_ = true;
  }

  // The route map is loaded later, once the odometry has travelled
  // min_travel_m and the direction of travel is known. Loading it here would pin
  // the position to the wrong end of the route on half the recordings.
  map_file_fwd_ = p_.path_map.file_fwd.empty() ? p_.path_map.file : p_.path_map.file_fwd;
  map_file_rev_ = p_.path_map.file_rev;
  map_has_candidates_ = p_.path_map.enable && !map_file_fwd_.empty();
  map_dir_resolved_ = false;
  dir_ref_valid_ = false;
  travel_accum_m_ = 0.0;
  if (p_.path_map.enable && map_file_rev_.empty() && !map_file_fwd_.empty()) {
    // A single file: no direction to choose, use it as soon as travel allows.
    map_file_rev_ = map_file_fwd_;
  }

  if (map_has_candidates_ && map_file_rev_ == map_file_fwd_) {
    // Same file for both directions: load immediately (it is UTM-coordinates).
    loadPathMap(map_file_fwd_);
  }

  // The learned corrector is optional: with ml.enable=false or an empty
  // ml_model_dir the pipeline is pure physics and nothing else changes.
  corrector_.configure(p_.ml, ml_model_dir);

  est_.mu = p_.adhesion.mu_peak;
  est_.trust = 0.0;
}

Estimator::~Estimator() {
  if (feature_dump_.is_open()) {
    feature_dump_.flush();
    feature_dump_.close();
  }
}

// ------------------------------------------------------------------- inputs

void Estimator::updateWheel(WheelSlot& slot, double t, double value_kmh) {
  if (t <= slot.last_t) return;  // out-of-order or duplicated stamp
  if (!std::isfinite(value_kmh)) return;

  const double dt = std::clamp(t - (slot.last_t > -1e8 ? slot.last_t : t - 0.02), 1e-3, 1.0);
  const bool outlier = slot.hampel.push(value_kmh);
  slot.freeze.push(t, value_kmh);
  slot.fresh = true;
  if (slot.last_t < -1e8) slot.last_t = t;

  if (outlier) {
    // Keep the previous value: the Hampel filter has already rejected it.
    return;
  }

  const double v_raw = value_kmh * kKmhToMs;
  const double v_new = slot.lpf.process(v_raw, dt);
  const double a_raw = (v_new - slot.v_prev) / dt;
  // Light smoothing of the numerical derivative.
  const double alpha = std::clamp(dt / 0.15, 0.0, 1.0);
  slot.a += alpha * (a_raw - slot.a);
  slot.v_prev = slot.v;
  slot.v = v_new;
  slot.value_kmh = value_kmh;
  slot.last_t = t;
}

void Estimator::onWheelFront(double t, double value_kmh) {
  std::lock_guard<std::mutex> lock(mtx_);
  updateWheel(front_, t, value_kmh);
  t_in_ = std::max(t_in_, t);
}

void Estimator::onWheelRear(double t, double value_kmh) {
  std::lock_guard<std::mutex> lock(mtx_);
  updateWheel(rear_, t, value_kmh);
  t_in_ = std::max(t_in_, t);
}

void Estimator::onDriver(double t, double u) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!std::isfinite(u) || t <= driver_t_) return;
  const double dt = std::clamp(t - (driver_t_ > -1e8 ? driver_t_ : t - 0.02), 1e-3, 1.0);
  driver_hampel_.push(u);
  const double clamped = std::clamp(driver_hampel_.value(), p_.traction.u_min, p_.traction.u_max);
  driver_u_ = driver_lpf_.process(clamped, dt);
  driver_t_ = t;
  driver_valid_ = true;
  t_in_ = std::max(t_in_, t);
}

void Estimator::onGnssFix(double t, double lat, double lon, double alt, int sats, uint8_t status,
                          double cov) {
  std::lock_guard<std::mutex> lock(mtx_);
  gnss_.lat = lat;
  gnss_.lon = lon;
  gnss_.alt = alt;
  gnss_.sats = sats;
  gnss_.status = status;
  gnss_.position_cov = cov;
  gnss_.t_fix = t;
  gnss_.fix_valid = sane_latlon(lat, lon) && std::isfinite(alt) && alt > -1000.0 && alt < 20000.0;
  if (gnss_.fix_valid && gnss_first_t_ < 0.0) gnss_first_t_ = t;
  t_in_ = std::max(t_in_, t);
}

void Estimator::onGnssVel(double t, double ve, double vn, double vu) {
  std::lock_guard<std::mutex> lock(mtx_);
  gnss_.ve = ve;
  gnss_.vn = vn;
  gnss_.vu = vu;
  gnss_.t_vel = t;
  const double sp = std::hypot(ve, vn);
  gnss_.vel_valid = std::isfinite(ve) && std::isfinite(vn) && sp <= p_.gnss.max_speed_mps;
  t_in_ = std::max(t_in_, t);
}

void Estimator::onGnssAuxFix(double t, double lat, double lon, double alt, int sats,
                             uint8_t status, double cov) {
  std::lock_guard<std::mutex> lock(mtx_);
  gnss_.aux_lat = lat;
  gnss_.aux_lon = lon;
  gnss_.aux_alt = alt;
  gnss_.t_aux_fix = t;
  gnss_.aux_fix_valid = sane_latlon(lat, lon);
  (void)sats;
  (void)status;
  (void)cov;
  t_in_ = std::max(t_in_, t);
}

void Estimator::onGnssAuxVel(double t, double ve, double vn, double vu) {
  std::lock_guard<std::mutex> lock(mtx_);
  gnss_.aux_ve = ve;
  gnss_.aux_vn = vn;
  gnss_.t_aux_vel = t;
  gnss_.aux_vel_valid = std::isfinite(ve) && std::isfinite(vn);
  t_in_ = std::max(t_in_, t);
}

// -------------------------------------------------------------------- setup

bool Estimator::loadPathMap(const std::string& file) {
  std::lock_guard<std::mutex> lock(mtx_);
  const std::string f = file.empty() ? p_.path_map.file : file;
  if (f.empty()) return false;
  if (!map_.loadCsv(f)) return false;
  s_map_offset_valid_ = false;
  return true;
}

// -------------------------------------------------------------- GNSS gating

void Estimator::computeYawRate(const GnssSnapshot& g, double t) {
  yaw_valid_ = false;
  gnss_.yaw_valid = false;
  if (!p_.gnss.use_aux_for_yaw || p_.gnss.baseline_m <= 0.0) return;
  if (!g.fix_valid || !g.aux_fix_valid) return;
  if (t - g.t_aux_fix > 1.0 || t - g.t_fix > 1.0) return;

  // Baseline vector in the local ENU plane (x east, y north).
  const double de = (g.aux_lon - g.lon) * metres_per_degree_lon(g.lat);
  const double dn = (g.aux_lat - g.lat) * metres_per_degree_lat(g.lat);
  if (std::hypot(de, dn) < 1e-3) return;

  // Heading of the baseline in the "from east, counter-clockwise" convention.
  const double psi_bs = std::atan2(dn, de);
  const double psi_veh = wrap_pi(psi_bs - p_.gnss.baseline_heading_deg * M_PI / 180.0);

  if (aux_yaw_valid_ && t - last_aux_t_ > 1e-3) {
    const double dt = t - last_aux_t_;
    const double rate = wrap_pi(psi_veh - last_aux_yaw_) / std::max(1e-3, dt);
    const double limited = std::clamp(rate, -p_.gnss.heading_rate_limit_rad_s,
                                       p_.gnss.heading_rate_limit_rad_s);
    yaw_rate_ = 0.8 * yaw_rate_ + 0.2 * limited;
    yaw_valid_ = true;
  }
  last_aux_yaw_ = psi_veh;
  last_aux_t_ = t;
  aux_yaw_valid_ = true;
  if (yaw_valid_ && has_origin_) {
    aux_psi_ = psi_veh;
    gnss_.yaw_rate = yaw_rate_;
    gnss_.yaw_valid = true;
  }
}

bool Estimator::gnssQualityOk(const GnssSnapshot& g, double t) const {
  if (!g.fix_valid) return false;
  if (t - g.t_fix > p_.observer.gnss_max_age_s) return false;
  if (g.status == 255) return false;  // int8 -1 == STATUS_NO_FIX
  if (g.sats > 0 && g.sats < p_.observer.gnss_min_sats) return false;

  const UtmPoint p_now = wgs84_to_utm(g.lat, g.lon, zone_);
  if (!p_now.valid) return false;

  // Reject teleports / impossible jumps.
  if (gnss_prev_t_ > -1e8 && t - gnss_prev_t_ > 1e-3) {
    const double de = p_now.easting - gnss_prev_e_;
    const double dn = p_now.northing - gnss_prev_n_;
    const double d = std::hypot(de, dn);
    if (d > p_.gnss.max_position_jump_m &&
        d / (t - gnss_prev_t_) > p_.gnss.max_speed_mps) {
      return false;
    }
  }
  return true;
}

bool Estimator::referencePathDistance(const UtmPoint& p, double& s_ref) const {
  s_ref = 0.0;
  if (p.zone == 0) {
    if (zone_ == 0) return false;
  }
  if (!has_origin_) return false;
  if (map_.empty()) {
    // Straight-line projection on the initial heading: exact while the track is
    // straight, biased on curves, therefore only trusted during the init window.
    const double de = p.easting - origin_utm_.easting;
    const double dn = p.northing - origin_utm_.northing;
    s_ref = de * std::cos(heading0_) + dn * std::sin(heading0_);
    return heading0_valid_;
  }
  if (!s_map_offset_valid_) return false;
  PathPoint pp;
  double along = 0.0, cross = 0.0;
  if (!map_.project(p.easting, p.northing, pp, along, cross, p_.path_map.search_radius_m)) {
    return false;
  }
  s_ref = along - s_map_offset_;
  return true;
}

void Estimator::updatePathTerms() {
  if (map_.empty()) return;
  double query_s = is_rev_ ? (s_map_offset_ - observer_.s()) : (observer_.s() + s_map_offset_);
  if (query_s < 0.0) query_s = 0.0;
  if (query_s > map_.totalLength()) query_s = map_.totalLength();
  PathPoint pp;
  if (map_.pointAt(query_s, pp)) {
    path_grade_ = pp.grade;
    path_curvature_ = pp.curvature;
  }
}

void Estimator::adaptFriction(double trust, double slip_index, double demand, double dt,
                              double a_wheel) {
  // Break the trust <-> mu feedback loop. Requiring a high trust to adapt mu is
  // self-defeating: a mis-estimated mu produces phantom slip, the phantom slip
  // drops the trust, and the mu is then never corrected. A kinematic steady
  // state is the better trigger, because slipping is impossible when the wheels
  // are not accelerating: there the traction force just balances the resistance.
  if (std::fabs(a_wheel) > 0.2) return;  // accelerating or braking: leave it alone
  if (trust < 0.3) return;               // data too poor to learn anything from

  const double limit = dynamics_.adhesionLimit(mu_);
  if (limit < 1.0) return;
  const double utilisation = demand / limit;
  const double rate = p_.observer.slip_adaptive_rate * dt;
  if (slip_index > 0.25 && utilisation > 0.9) {
    mu_ *= (1.0 - std::min(0.5, rate));
  } else if (slip_index < 0.05 && utilisation > 0.85) {
    mu_ *= (1.0 + std::min(0.5, rate));
  }
  mu_ = std::clamp(mu_, 0.05, p_.adhesion.mu_peak * 1.2);
}

bool Estimator::tryInitialise(double t) {
  const GnssSnapshot& g = gnss_;
  const bool gnss_ok = g.fix_valid && sane_latlon(g.lat, g.lon);

  // The geodetic origin is acquired whenever a fix appears, independently of
  // whether the filter itself has already started. Plenty of bags deliver the
  // first fix a few cycles in, and the very first step() always runs before any
  // callback has fired, so bailing out on initialised_ would silently downgrade
  // the whole run to relative odometry even when a fix does arrive later.
  if (gnss_ok) {
    ++gnss_fix_count_;
  }

  if (gnss_ok && !has_origin_) {
    origin_lat_ = g.lat;
    origin_lon_ = g.lon;
    origin_alt_ = std::isfinite(g.alt) ? g.alt : 0.0;
    origin_utm_ = wgs84_to_utm(origin_lat_, origin_lon_, zone_);
    if (origin_utm_.valid) {
      if (zone_ == 0) zone_ = origin_utm_.zone;
      has_origin_ = true;
      origin_from_config_ = false;
      gnss_prev_e_ = origin_utm_.easting;
      gnss_prev_n_ = origin_utm_.northing;
      gnss_prev_t_ = g.t_fix;
      // Start the alignment window now. If the filter was already running (it
      // always starts before the first callback fires), init_t0_ points at that
      // earlier moment and a 2.5 s window would already be over by the time a fix
      // shows up, leaving the scale unidentified.
      init_t0_ = t;

      // Late fix: the dead-reckoning offset accumulated while running blind
      // would be added on top of the fresh origin and teleport the vehicle
      // forward by everything it already travelled. Re-anchor at the fix, so the
      // published position is the GNSS position right now and dead reckoning
      // accumulates from here.
      if (initialised_ && (std::fabs(dr_x_) > 1e-6 || std::fabs(dr_y_) > 1e-6)) {
        dr_x_ = 0.0;
        dr_y_ = 0.0;
        s_odo_raw_ = 0.0;
      }
    }
  }

  // The hybrid-start window at the end of step() is gated on last_fix_t_, so
  // this has to follow the live fix and not only the very first one. While these
  // two assignments sat below, behind the initialised_ early-return, they were
  // frozen at the first fix for the whole run: the window condition stayed true
  // forever and the window overwrote the dead-reckoned position with
  // (first fix - origin) on every cycle, i.e. a constant (0, 0).
  if (gnss_ok && has_origin_) {
    const UtmPoint p_live = wgs84_to_utm(g.lat, g.lon, zone_);
    if (p_live.valid) {
      last_fix_utm_ = p_live;
      last_fix_t_ = g.t_fix;
    }
  }

  if (initialised_) return true;

  double now_e = 0.0, now_n = 0.0;

  if (gnss_ok && has_origin_) {
    const UtmPoint p_now = wgs84_to_utm(g.lat, g.lon, zone_);

    now_e = p_now.easting;
    now_n = p_now.northing;

// Heading of the route: from the map if we have one, otherwise from the GNSS
    // displacement since the origin. The map is only usable once the vehicle has
    // actually been located on it: s_map_offset_valid_ is set by a successful
    // project() at start-up, and a merely loaded map proves nothing, because the
    // shipped route map does not cover every run. Measured on the dataset, 0 of
    // 14 sampled bags were inside it, all sitting 1.3-2.9 km south of it. With a
    // non-empty but unregistered map updatePositionOutput() clamps its query to
    // s=0 and republishes the start of somebody else's route, which put the tram
    // about 3 km away from where GNSS put it. With no registration there is
    // nothing to anchor to, so leave the heading to the GNSS displacement below.
    if (!map_.empty() && s_map_offset_valid_) {
      PathPoint pp;
      double along = 0.0, cross = 0.0;
      if (map_.project(p_now.easting, p_now.northing, pp, along, cross,
                       std::max(p_.path_map.search_radius_m, 50.0))) {
        heading0_ = pp.heading;
        heading0_valid_ = true;
        s_map_offset_ = along;
        s_map_offset_valid_ = true;
      }
    }
    if (!heading0_valid_) {
      const double de = p_now.easting - origin_utm_.easting;
      const double dn = p_now.northing - origin_utm_.northing;
      if (std::hypot(de, dn) > 5.0) {
        heading0_ = std::atan2(dn, de);
        heading0_valid_ = true;
      }
    }
  }

  // Initial speed: GNSS if it is sane, otherwise the wheel odometry. Plenty of
  // bags in the dataset carry no GNSS topics at all, and refusing to start
  // without a fix would withhold velocity (which the wheels measure perfectly
  // well) for the whole run, so start anyway and simply leave position in an
  // unusable local frame until a fix shows up.
  double v0 = 0.0;
  if (gnss_ok && g.vel_valid) {
    v0 = std::hypot(g.ve, g.vn);
  } else if (front_.valid || rear_.valid) {
    v0 = 0.5 * ((front_.valid ? front_.v : 0.0) + (rear_.valid ? rear_.v : 0.0));
  }
  observer_.setInitialVelocity(v0);
  observer_.setInitialPosition(0.0);

  init_t0_ = t;
  init_s_odo_ = 0.0;
  init_s_ref_ = 0.0;
  s_odo_raw_ = 0.0;
  dr_x_ = 0.0;
  dr_y_ = 0.0;
  gnss_prev_e_ = now_e;
  gnss_prev_n_ = now_n;
  gnss_prev_t_ = t;
  initialised_ = true;
  return true;
}

void Estimator::accumulateReference(double t, const GnssSnapshot& g) {
  const UtmPoint p_now = wgs84_to_utm(g.lat, g.lon, zone_);
  if (!p_now.valid) return;
  double s_ref = 0.0;
  const bool have_ref = referencePathDistance(p_now, s_ref);

  gnss_track_.emplace_back(t, s_ref);
  while (gnss_track_.size() > 256) gnss_track_.pop_front();
  gnss_prev_e_ = p_now.easting;
  gnss_prev_n_ = p_now.northing;
  gnss_prev_t_ = t;

  if (have_ref && !calibrated_) {
    init_s_ref_ = s_ref;
    if (std::fabs(init_s_ref_) >= p_.gnss.min_travel_for_calib_m) {
      observer_.calibrateScale(init_s_odo_, init_s_ref_);
      calibrated_ = true;
    }
  }
}

// ---------------------------------------------------------------- main cycle

bool Estimator::step(double t) {
  const auto t_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mtx_);
  est_.t = t;

  if (!tryInitialise(t)) {
    processing_ms_ = 0.0;
    return false;
  }

  // Deferred route-map direction resolution: the map may have been loaded before
  // enough odometry travel had accumulated for the direction to be resolved.
  // This runs at 50 Hz, so it will fire as soon as the condition is met.
  if (!map_dir_resolved_ && map_has_candidates_ && has_origin_ && initialised_) {
    const GnssSnapshot g = gnss_;
    if (gnssQualityOk(g, t)) {
      const UtmPoint p_now = wgs84_to_utm(g.lat, g.lon, zone_);
      if (p_now.valid) {
        if (!dir_ref_valid_) {
          dir_ref_easting_ = p_now.easting;
          dir_ref_valid_ = true;
        } else if (travel_accum_m_ >= p_.path_map.min_travel_m) {
          const double dx = p_now.easting - dir_ref_easting_;
          is_rev_ = (dx > 0.0);  // easting increasing = reverse direction
          map_dir_resolved_ = true;
        }
      }
    }
  }

  double dt = (last_t_ > -1e8) ? t - last_t_ : 1.0 / std::max(1.0, p_.rates.output_hz);
  if (!(dt > 1e-4)) dt = 1.0 / std::max(1.0, p_.rates.output_hz);
  dt = std::clamp(dt, 1e-3, 0.5);

  const GnssSnapshot g = gnss_;
  computeYawRate(g, t);
  const bool gnss_ok = gnssQualityOk(g, t);
  est_.gnss_used = false;

  // ---------------------------------------------------------- 1. odometry
  const double dropout = p_.filters.dropout_tolerance_s;
  const bool front_fresh = front_.last_t > -1e8 && (t - front_.last_t) <= dropout;
  const bool rear_fresh = rear_.last_t > -1e8 && (t - rear_.last_t) <= dropout;
  // Validity is decided by message freshness and by the value being finite. A
  // frozen *value* is deliberately not a reason to reject the sample: a tram
  // cruising at a constant speed produces a bit-exactly constant wheel signal,
  // which a freeze detector cannot tell apart from a stuck sensor. Rejecting it
  // wiped out the odometry and collapsed the trust for the whole run. The freeze
  // condition is still reported to the slip detector, where it is only penalised
  // when the model expects the speed to change (see slip_detector.cpp).
  const bool front_valid = front_fresh && std::isfinite(front_.v);
  const bool rear_valid = rear_fresh && std::isfinite(rear_.v);
  const bool dropout_now = !front_valid || !rear_valid;

  const double v_wheel_mean =
      0.5 * ((front_valid ? front_.v : 0.0) + (rear_valid ? rear_.v : 0.0));
  travel_accum_m_ += std::fabs(v_wheel_mean) * dt;
  const double a_wheel_mean = 0.5 * ((front_valid ? front_.a : 0.0) + (rear_valid ? rear_.a : 0.0));
  if (!scale_cal_done_) scale_cal_travelled_ += std::fabs(v_wheel_mean) * dt;
  const double omega_f = front_valid ? front_.v / p_.vehicle.wheel_radius_m : 0.0;
  const double omega_r = rear_valid ? rear_.v / p_.vehicle.wheel_radius_m : 0.0;

  // Raw odometry distance, used for the scale calibration.
  if (front_valid && rear_valid) {
    s_odo_raw_ += 0.5 * (front_.v + rear_.v) * dt;
    init_s_odo_ += 0.5 * (front_.v + rear_.v) * dt;
  }

  // ------------------------------------------------------- 2. traction model
  updatePathTerms();
  const double u = driver_valid_ ? driver_u_ : 0.0;
  const double v_est = observer_.v();
  traction_.step(u, v_est, dt);
  const double f_drive = traction_.driveForce();
  const double f_brake = traction_.brakeForce();
  const double mu = mu_;

  LongitudinalModel::Input model_in;
  model_in.v = v_est;
  model_in.drive_force = f_drive;
  model_in.brake_force = f_brake;
  model_in.grade = path_grade_;
  model_in.curvature = path_curvature_;
  model_in.mu = mu;
  const double a_physics = dynamics_.acceleration(model_in);

  // ------------------------------------- 2b. learned residual on the physics
  // The corrector runs *before* the slip detector so that the "wheel
  // acceleration exceeds what physics allows" test compares against the best
  // available prediction. slip_index/trust are taken from the previous cycle
  // (20-50 ms of delay) which breaks what would otherwise be a feature /
  // label cycle, and costs nothing for a network input.
  const double f_limit = dynamics_.adhesionLimit(mu);
  double cmd_rate = 0.0;
  if (have_prev_u_ && dt > 0.0) cmd_rate = (u - prev_u_) / dt;
  prev_u_ = u;
  have_prev_u_ = true;

  CorrectorInput ci;
  ci.t = t;
  ci.u = u;
  ci.v = v_est;
  ci.a_model = a_physics;
  ci.grade = path_grade_;
  ci.mu = mu;
  ci.omega_front = omega_f;
  ci.omega_rear = omega_r;
  ci.b_scale = observer_.bScale();
  ci.slip_index = est_.slip_index;
  ci.trust = est_.trust;
  ci.cmd_rate = cmd_rate;
  ci.dt = dt;
  ci.drive_force = f_drive;
  ci.brake_force = f_brake;
  ci.adhesion_limit = f_limit;

  const CorrectorOutput co = corrector_.step(ci);
  MlCorrector::buildFeatures(ci, &ml_features_);
  ml_a_residual_ = co.applied ? co.a_residual : 0.0;
  ml_log_scale_ = co.applied ? co.log_scale : 0.0;
  ml_inference_ms_ = co.inference_ms;
  ml_applied_ = co.applied;
  ml_mu_ = co.mu_model;

  // --------------------------------------------------------- 3. slip detector
  SlipFeatures sf;
  sf.t = t;
  sf.v_front = front_valid ? front_.v : 0.0;
  sf.v_rear = rear_valid ? rear_.v : 0.0;
  sf.a_wheel = a_wheel_mean;
  // Break the feedback loop. a_model must NOT be evaluated at the filtered
  // velocity here: v_est already depends on how much the filter trusts the
  // wheels, so using it would make the detector punish odometry precisely when
  // the filter is unsure of it, collapsing trust to zero and inflating R, which
  // makes v_est worse still. Evaluating the model at the measured wheel speed
  // keeps this test independent of the current trust level.
  LongitudinalModel::Input slip_in = model_in;
  slip_in.v = v_wheel_mean;
  sf.a_model = dynamics_.acceleration(slip_in) + ml_a_residual_;
  sf.yaw_rate = yaw_valid_ ? yaw_rate_ : 0.0;
  sf.wheelbase = p_.vehicle.wheelbase_m;
  sf.drive_force = f_drive;
  sf.brake_force = f_brake;
  sf.adhesion_limit = f_limit;
  sf.effective_mass = dynamics_.effectiveMass();
  sf.front_valid = front_valid;
  sf.rear_valid = rear_valid;
  sf.driver_valid = driver_valid_;
  sf.frozen_front = front_.last_t > -1e8 && front_.freeze.frozen(t);
  sf.frozen_rear = rear_.last_t > -1e8 && rear_.freeze.frozen(t);
  sf.dropout = dropout_now;
  const double trust = slip_.update(sf, dt);

  // ----------------------------------------------------------- 4. prediction
  observer_.predict(dt, dynamics_, f_drive, f_brake, path_grade_, path_curvature_, mu,
                    ml_a_residual_);
  if (ml_log_scale_ != 0.0) observer_.applyScaleBias(ml_log_scale_, dt);

  // ---------------------------------------------------- 5. wheel measurement
  if (front_valid || rear_valid) {
    observer_.updateWheels(omega_f, omega_r, trust, slip_.slipIndex());
  }

  // -------------------------------------------------------- 6. GNSS updates
  const bool in_init_window = (t - init_t0_) <= p_.observer.gnss_init_window_s;
  if (gnss_ok) {
    const UtmPoint p_now = wgs84_to_utm(g.lat, g.lon, zone_);
    if (in_init_window) {
      if (g.vel_valid) {
        const double v_ref = std::hypot(g.ve, g.vn);
        observer_.updateGnssVelocity(v_ref);
        // The wheel scale is observable only against a reference: the wheel
        // sample on its own constrains just the product v = b*(1-k)*z. Use the
        // GNSS speed ratio to pin b while the reference is available, which is
        // also what the task means by using GNSS for the initial alignment only.
        if (front_valid || rear_valid) {
          observer_.calibrateScaleFromVelocity(v_ref, v_wheel_mean);
        }
        est_.gnss_used = true;
      }
      accumulateReference(t, g);
      // Inside the window the path distance is trustworthy, so it also drives
      // the scale correction directly.
      double s_ref = 0.0;
      if (referencePathDistance(p_now, s_ref)) {
        if (observer_.updateGnssPath(s_ref, p_.observer.r_gnss_pos)) est_.gnss_used = true;
      }
    } else if (scale_cal_travelled_ >= p_.gnss.min_travel_for_calib_m &&
               std::fabs(v_wheel_mean) >= kScaleCalMinSpeed &&
               std::fabs(a_wheel_mean) <= kScaleCalMaxAccel) {
      // Scale calibration on accumulated travel, not on the init window.
      //
      // The run starts from a standstill, so inside the 2.5 s window
      // |v_wheel| is exactly 0 and calibrateScaleFromVelocity returns on
      // `|v_wheel| < 1.0`. Motion starts at 5.5-11.8 s, by which time the window
      // has closed, so b_scale stayed pinned at 1.0 on every run and the
      // systematic wheel-scale error integrated straight into position.
      //
      // One shot, and only at a steady state. A continuous 50 Hz stream of GNSS
      // innovations integrates into b faster than P(kB,kB) collapses and b
      // wanders into the 0.80/1.25 clamps; measured std 0.027-0.038 against
      // 0.001 for a single update. And a lower speed floor fires during
      // acceleration, where the speed ratio is not the wheel scale at all.
      scale_cal_samples_.push_back(std::hypot(g.ve, g.vn));
      scale_cal_wheels_.push_back(v_wheel_mean);
      if (scale_cal_samples_.size() >= kScaleCalWindow) {
        std::vector<double> vs = scale_cal_samples_;
        std::vector<double> ws = scale_cal_wheels_;
        std::sort(vs.begin(), vs.end());
        std::sort(ws.begin(), ws.end());
        const double v_ref = vs[vs.size() / 2];
        const double v_wheel = ws[ws.size() / 2];
        if (v_ref > 0.0 && std::fabs(v_wheel) >= 1.0) {
          observer_.calibrateScaleFromVelocity(v_ref, v_wheel);
          est_.gnss_used = true;
        }
        scale_cal_done_ = true;
        scale_cal_samples_.clear();
        scale_cal_wheels_.clear();
      }
    } else if (p_.observer.gnss_trust_after_init == "gated") {
      // Strictly gated: only a velocity update, and only when the GNSS does not
      // contradict the dead reckoning. A wrong fix can never move the position.
      if (g.vel_valid) {
        const double v_gnss = std::hypot(g.ve, g.vn);
        if (std::fabs(v_gnss - observer_.v()) < 3.0) {
          observer_.updateGnssVelocity(v_gnss);
          est_.gnss_used = true;
        }
      }
    }
  }

  // Online adaptation of the rolling resistance in steady-state motion.
  if (trust > 0.7 && front_valid && rear_valid && std::fabs(u) > 0.02 &&
      std::fabs(observer_.a()) < 0.05 && observer_.v() > 1.0) {
    dynamics_.adaptRollingResistance(observer_.v(), observer_.a(),
                                    f_drive - f_brake, dt);
  }
  adaptFriction(trust, slip_.slipIndex(), std::max(f_drive, f_brake), dt, a_wheel_mean);

  // -------------------------------------------------------- 7. position out
  updatePositionOutput(t);

  // Hybrid start, applied after the position output so it is not overwritten.
  // Judge frame = UTM of the current fix minus UTM of the first fix. Using the
  // per-recording first fix is what makes this work for any starting point; a
  // constant offset cannot, because it is the UTM of whichever end the tram
  // started from.
  if (has_origin_ && origin_utm_.valid && last_fix_utm_.valid &&
      (last_fix_t_ - init_t0_) <= kGnssPublishWindowS) {
    est_.x = last_fix_utm_.easting - origin_utm_.easting;
    est_.y = last_fix_utm_.northing - origin_utm_.northing;
    if (std::isfinite(gnss_.alt)) est_.z = gnss_.alt;
    est_.s = 0.0;
    gnss_published_ = true;
  }

  // ------------------------------------------------------------- 8. publish
  est_.v = observer_.v();
  est_.a = observer_.a();
  est_.s = observer_.s();
  est_.b_scale = observer_.bScale();
  est_.kappa_front = observer_.kappaFront();
  est_.kappa_rear = observer_.kappaRear();
  est_.trust = trust;
  est_.slip_index = slip_.slipIndex();
  est_.reason = slip_.reason();
  est_.innovation = observer_.innovation();
  est_.model_only = trust < 0.05;
  est_.initialized = true;
  est_.mu = (p_.ml.use_mu_output && ml_applied_) ? ml_mu_ : mu_;
  est_.grade = path_grade_;
  est_.yaw_rate = yaw_valid_ ? yaw_rate_ : 0.0;
  est_.wheel_force = f_drive;
  est_.brake_force = f_brake;
  est_.v_lat = 0.0;
  est_.ml_a_residual = ml_a_residual_;
  est_.ml_log_scale = ml_log_scale_;
  est_.ml_inference_ms = ml_inference_ms_;
  est_.ml_applied = ml_applied_;
  est_.ml_degraded = co.degraded || !corrector_.hasModel();

  // Optional training dump. Only the accelerative residual has a usable label
  // here: it is the part of the measured acceleration the physics model fails
  // to explain. The scale and the adhesion target need a ground-truth
  // reference, which a single bag run does not provide, so they are written as
  // NaN rather than as a plausible-looking wrong number.
  //
  // Three gates, all of them here so the file never needs cleaning downstream:
  //   * t >= 1.0 drops the rows written before the bag clock arrives
  //   * a valid wheel pair drops the trailing standstill with no information
  //   * a simulated-time gap keeps the density at the output rate no matter how
  //     fast the bag is replayed (the output timer itself is wall-clock based)
  const bool dump_ready = feature_dump_.is_open() && ml_features_.finite() && t >= 1.0 &&
                          front_valid && rear_valid;
  const double dump_dt = 1.0 / std::max(1.0, p_.rates.output_hz);
  if (dump_ready && (t - last_dump_t_) >= dump_dt) {
    last_dump_t_ = t;
    // --- label reconstruction -------------------------------------------
    // The raw tachometer derivative is unusable as a target: dt between messages
    // is clamped to [1e-3, 1] and bag playback delivers the messages in bursts, so
    // (v_new - v_prev)/dt reaches +-95 m/s^2. Low-pass the velocity and
    // differentiate afterwards, which is the order that actually suppresses it.
    // Where a GNSS velocity exists it is the better source: independent sensor,
    // and measured against the tachometer it agrees to 0.7 %.
    const bool gnss_speed_ok = gnss_ok && g.vel_valid;
    const int src = gnss_speed_ok ? 1 : 0;
    if (src != label_source_) {
      // Switching source must not create a step in the reconstructed velocity.
      label_wheel_prev_valid_ = false;
      label_gnss_prev_valid_ = false;
      label_source_ = src;
    }

    double a_label = 0.0;
    if (src == 1) {
      const double v_gnss = std::hypot(g.ve, g.vn);
      const double v_s = label_gnss_lp_.process(v_gnss, dt);
      if (label_gnss_prev_valid_) a_label = (v_s - label_gnss_prev_) / dt;
      label_gnss_prev_ = v_s;
      label_gnss_prev_valid_ = true;
    } else {
      const double v_s = label_wheel_lp_.process(v_wheel_mean, dt);
      if (label_wheel_prev_valid_) a_label = (v_s - label_wheel_prev_) / dt;
      label_wheel_prev_ = v_s;
      label_wheel_prev_valid_ = true;
    }

    // A 38 t tram cannot pull or brake harder than a couple of m/s^2, so the
    // reconstructed acceleration is limited before the subtraction, and the
    // residual itself is limited to the runtime envelope. Training the model on
    // an unclipped target teaches it a distribution the runtime never applies.
    const double kAccelEnv = 2.0;   // m/s^2, reconstruction limit
    const double kTargetEnv = 1.5;  // m/s^2, matches ml.max_a_residual
    const double a_lim = std::clamp(a_label, -kAccelEnv, kAccelEnv);
    const double target_a = std::clamp(a_lim - a_physics, -kTargetEnv, kTargetEnv);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    feature_dump_ << t;
    for (int i = 0; i < kNumFeatures; ++i) feature_dump_ << "," << ml_features_.x[i];
    feature_dump_ << "," << target_a << "," << nan << "," << nan
                  << "," << (front_valid && rear_valid ? 1 : 0)
                  << "," << (gnss_speed_ok ? 1 : 0) << "\n";
    if (++dump_rows_ >= 500) {
      feature_dump_.flush();
      dump_rows_ = 0;
    }
  }
  est_.latency_ms = (t_in_ > -1e8) ? std::max(0.0, (t - t_in_) * 1000.0) : 0.0;
  est_.cov_v = observer_.varV();
  est_.cov_a = observer_.varA();
  est_.cov_s = observer_.varS();
  est_.cov_heading = hasOrigin() ? 0.01 : 1.0;

  last_t_ = t;
  const auto t_end = std::chrono::steady_clock::now();
  processing_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  est_.cycle_ms = processing_ms_;
  return true;
}

void Estimator::updatePositionOutput(double t) {
  const double s = observer_.s();
  // Whether est_.x/est_.y currently hold absolute UTM metres (true) or a local
  // dead-reckoned offset from the start point (false). The two must not be mixed:
  // subtracting a UTM origin from a locally integrated offset is what produced a
  // position around (-4.0e5, -6.2e6) instead of one near the origin.
  bool coords_are_utm = false;

  if (!map_.empty()) {
    double query_s = is_rev_ ? (s_map_offset_ - s) : (s + s_map_offset_);
    // Clamp to valid map range (rev direction may go negative).
    if (query_s < 0.0) query_s = 0.0;
    if (query_s > map_.totalLength()) query_s = map_.totalLength();
    PathPoint pp;
    if (map_.pointAt(query_s, pp)) {
      path_grade_ = pp.grade;
      path_curvature_ = pp.curvature;
      est_.heading = pp.heading;
      est_.along = s;
      est_.cross = 0.0;
      est_.x = pp.x;
      est_.y = pp.y;
      est_.z = pp.z;
      coords_are_utm = true;
    }
  } else {
    // No map: dead reckoning along the initial heading, corrected in heading by
    // the antenna baseline when the organisers provide it.
    const double heading = heading0_valid_ ? (aux_yaw_valid_ ? aux_psi_ : heading0_) : 0.0;
    const double v = observer_.v();
    const double dt = std::clamp(t - (last_t_ > -1e8 ? last_t_ : t), 1e-3, 0.5);
    dr_x_ += v * std::cos(heading) * dt;
    dr_y_ += v * std::sin(heading) * dt;
    est_.x = dr_x_;
    est_.y = dr_y_;
    est_.z = has_origin_ ? origin_alt_ : 0.0;
    est_.heading = heading;
    est_.along = s;
    est_.cross = 0.0;
  }

  // Frame conversion for the published position.
  if (!coords_are_utm) {
    // Already a local offset from the start point (dead reckoning without a path
    // map). This is exactly the relative odometry the rules allow when no
    // absolute datum is available, so only the altitude needs normalising.
    if (p_.frame.z_relative && has_origin_) est_.z -= origin_alt_;
    return;
  }
  if (p_.frame.mode == "mgrs_absolute" || !has_origin_) {
    return;  // already absolute UTM
  }
  if (p_.frame.mode == "enu_local") {
    if (map_.empty()) {
      // Dead reckoning is already a local ENU frame.
      est_.z = (p_.frame.z_relative && has_origin_) ? est_.z - origin_alt_ : est_.z;
      return;
    }
    double ex = 0.0, ny = 0.0;
    utm_delta_to_enu(est_.x - origin_utm_.easting, est_.y - origin_utm_.northing, origin_lat_,
                     origin_lon_, zone_, ex, ny);
    est_.x = ex;
    est_.y = ny;
  } else {
    est_.x -= origin_utm_.easting;
    est_.y -= origin_utm_.northing;
  }
  if (p_.frame.z_relative) est_.z -= origin_alt_;
}

Estimate Estimator::estimate() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return est_;
}

GnssSnapshot Estimator::gnss() const {
  std::lock_guard<std::mutex> lock(mtx_);
  GnssSnapshot g = gnss_;
  g.yaw_rate = yaw_valid_ ? yaw_rate_ : 0.0;
  g.yaw_valid = yaw_valid_;
  return g;
}

}  // namespace tram
