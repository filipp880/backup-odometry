// Top-level pipeline: preprocessing -> traction model -> slip detection ->
// ESKF -> position assembly. ROS-free on purpose so it can be unit tested and
// profiled outside the node.
#pragma once

#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "tram_odometry/geo.hpp"
#include "tram_odometry/longitudinal_model.hpp"
#include "tram_odometry/ml_corrector.hpp"
#include "tram_odometry/observer.hpp"
#include "tram_odometry/params.hpp"
#include "tram_odometry/path_map.hpp"
#include "tram_odometry/signal_filter.hpp"
#include "tram_odometry/slip_detector.hpp"
#include "tram_odometry/traction_model.hpp"
#include "tram_odometry/types.hpp"

namespace tram {

/// Latest GNSS picture, used for initialisation, calibration and diagnostics.
struct GnssSnapshot {
  bool fix_valid = false;
  double lat = 0.0, lon = 0.0, alt = 0.0;
  double t_fix = 0.0;
  int sats = 0;
  uint8_t status = 0;
  double position_cov = -1.0;  ///< horizontal position covariance, -1 = not reported
  bool vel_valid = false;
  double ve = 0.0, vn = 0.0, vu = 0.0;
  double t_vel = 0.0;
  bool aux_fix_valid = false;
  double aux_lat = 0.0, aux_lon = 0.0, aux_alt = 0.0;
  double aux_ve = 0.0, aux_vn = 0.0;
  double t_aux_fix = 0.0, t_aux_vel = 0.0;
  bool aux_vel_valid = false;
  double yaw_rate = 0.0;
  bool yaw_valid = false;
};

class Estimator {
 public:
  /// `ml_model_dir` is only used when ml.enable is true and ml.model_dir is
  /// empty; the node passes the package share directory there.
  explicit Estimator(const Params& params, const std::string& ml_model_dir = std::string());
  ~Estimator();

  // ------------------------------------------------------------- input side
  // All callbacks are thread safe (they are called from ROS executor threads).
  void onWheelFront(double t, double value_kmh);
  void onWheelRear(double t, double value_kmh);
  void onDriver(double t, double u);
  void onGnssFix(double t, double lat, double lon, double alt, int sats, uint8_t status,
                 double cov);
  void onGnssVel(double t, double ve, double vn, double vu);
  void onGnssAuxFix(double t, double lat, double lon, double alt, int sats, uint8_t status,
                    double cov);
  void onGnssAuxVel(double t, double ve, double vn, double vu);

  // ------------------------------------------------------------ output side
  /// One control cycle. Returns false until the state is initialised.
  bool step(double t);
  Estimate estimate() const;
  GnssSnapshot gnss() const;
  double processingMs() const { return processing_ms_; }

  bool loadPathMap(const std::string& file);
  bool hasPathMap() const { return !map_.empty(); }
  double pathLength() const { return map_.totalLength(); }
  bool hasOrigin() const { return has_origin_; }
  UtmPoint originUtm() const { return origin_utm_; }
  int utmZone() const { return zone_; }
  bool originFromConfig() const { return origin_from_config_; }
  /// Cumulative odometry distance (raw, before scale calibration).
  double odometryDistance() const { return s_odo_raw_; }

  // ------------------------------------------------------- learned corrector
  const MlCorrector& corrector() const { return corrector_; }
  /// Feature vector the corrector sees this cycle; exposed so the Python side
  /// can reproduce the exact training inputs from a rosbag.
  const FeatureVector& lastFeatures() const { return ml_features_; }

 private:
  struct WheelSlot {
    HampelFilter hampel{7, 3.0};
    LowPassFilter lpf{25.0, 0.02};
    FreezeDetector freeze{0.7};
    double last_t = -1e9;
    double value_kmh = 0.0;
    double v = 0.0;         ///< filtered, m/s
    double v_prev = 0.0;
    double a = 0.0;         ///< filtered derivative, m/s^2
    bool valid = false;
    bool fresh = false;
  };

  void updateWheel(WheelSlot& slot, double t, double value_kmh);
  bool tryInitialise(double t);
  bool gnssQualityOk(const GnssSnapshot& g, double t) const;
  void accumulateReference(double t, const GnssSnapshot& g);
  void updatePositionOutput(double t);
  /// Reference distance travelled, measured by GNSS. False when unavailable.
  bool referencePathDistance(const UtmPoint& p, double& s_ref) const;
  void updatePathTerms();
  void adaptFriction(double trust, double slip_index, double demand, double dt,
                     double a_wheel);
  void computeYawRate(const GnssSnapshot& g, double t);

  Params p_;
  TractionModel traction_;
  LongitudinalModel dynamics_;
  SlipDetector slip_;
  Observer observer_;
  PathMap map_;
  MlCorrector corrector_;

  mutable std::mutex mtx_;
  WheelSlot front_;
  WheelSlot rear_;
  HampelFilter driver_hampel_{7, 4.0};
  LowPassFilter driver_lpf_{10.0, 0.02};
  double driver_u_ = 0.0;
  double driver_t_ = -1e9;
  bool driver_valid_ = false;

  GnssSnapshot gnss_;
  std::deque<std::pair<double, double>> gnss_track_;  ///< (t, along-reference distance)
  double gnss_first_t_ = -1.0;
  double gnss_last_heading_ = 0.0;
  bool gnss_heading_valid_ = false;

  // origin of the output frame
  bool has_origin_ = false;
  bool origin_from_config_ = false;
  double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_alt_ = 0.0;
  UtmPoint origin_utm_;
  int zone_ = 0;

  // initialisation / calibration
  bool initialised_ = false;
  double init_t0_ = 0.0;
  double init_s_odo_ = 0.0;
  double init_s_ref_ = 0.0;
  double s_odo_raw_ = 0.0;
  bool calibrated_ = false;
  double heading0_ = 0.0;
  bool heading0_valid_ = false;
  double dr_x_ = 0.0, dr_y_ = 0.0;  ///< dead-reckoning position (no map case)
  double yaw_rate_ = 0.0;
  bool yaw_valid_ = false;
  double last_aux_yaw_ = 0.0;
  double last_aux_t_ = -1e9;
  bool aux_yaw_valid_ = false;
  double mu_ = 0.35;

  double last_t_ = -1e9;
  double t_in_ = -1e9;  ///< timestamp of the freshest input message
  double processing_ms_ = 0.0;
  Estimate est_;
  double path_grade_ = 0.0;
  double path_curvature_ = 0.0;
  // learned corrector state
  FeatureVector ml_features_;
  /// Optional CSV dump of the exact feature vector handed to the corrector, so
  /// the learned model can be trained on the states it will actually see. The
  /// columns are written from ml_features_, never recomputed, which guarantees
  /// the order and the values match kFeatureNames bit for bit.
  std::ofstream feature_dump_;
  int dump_rows_ = 0;
  /// The output timer is wall-clock based, so replaying a bag at --rate 10 would
  /// thin the dump out by the same factor. Gating on simulated time instead keeps
  /// the density at the configured output rate no matter how fast the bag plays,
  /// which is what makes a high replay rate actually buy extra rows.
  double last_dump_t_ = -1e9;
  /// Velocity reconstruction used ONLY to build the training label. The
  /// tachometer derivative is far too noisy to learn from, so the velocity is
  /// low-passed first and differentiated afterwards. GNSS velocity is preferred
  /// where it exists because it is an independent sensor; the tachometer is the
  /// fallback. Separate filters per source so a switch cannot inject a step.
  LowPassFilter label_wheel_lp_{1.0, 0.02};
  LowPassFilter label_gnss_lp_{1.0, 0.02};
  double label_wheel_prev_ = 0.0;
  double label_gnss_prev_ = 0.0;
  bool label_wheel_prev_valid_ = false;
  bool label_gnss_prev_valid_ = false;
  int label_source_ = -1;  ///< -1 unknown, 0 tachometer, 1 GNSS
  double ml_a_residual_ = 0.0;
  double ml_log_scale_ = 0.0;
  double ml_inference_ms_ = 0.0;
  double ml_mu_ = 0.0;

  // --- wheel-scale calibration on accumulated travel -------------------------
  // See the block in estimator.cpp for why the 2.5 s init window cannot work:
  // the run starts from a standstill, so |v_wheel| is 0 while the window is open.
  // --- route map: deferred load, direction resolved from travel ------------
  // The map is not loaded at construction. The judge localisation is ~500 m off
  // the route for the first 220 s, so the run must be blind until the odometry
  // has actually travelled far enough to tell which way along the route it is
  // going. Direction comes from the sign of the UTM easting change, measured
  // between the first fix and the moment min_travel_m has accumulated.
  std::string map_file_fwd_, map_file_rev_;
  double travel_accum_m_ = 0.0;
  bool map_dir_resolved_ = false;
  bool map_has_candidates_ = false;
  bool is_rev_ = false;  /**< tram travelling in reverse direction on route map */
  double dir_ref_easting_ = 0.0;
  bool dir_ref_valid_ = false;

  // --- hybrid start --------------------------------------------------------
  // For the first GNSS_PUBLISH_WINDOW_S seconds the published position comes
  // straight from the GNSS fix, expressed in the judge frame as UTM minus the
  // UTM of the first fix. It has to: the judge localisation sits ~500 m off the
  // route for the first 220 s of the run, so there is nothing trustworthy to
  // match in that window, and the map cannot be applied before the odometry has
  // shown which way the tram is travelling.
  UtmPoint last_fix_utm_;
  double last_fix_t_ = 0.0;
  uint64_t gnss_fix_count_ = 0;
  bool gnss_published_ = false;

 public:
  bool mapDirResolved() const { return map_dir_resolved_; }
  double travelAccumM() const { return travel_accum_m_; }
  uint64_t gnssFixCount() const { return gnss_fix_count_; }
  bool gnssPublished() const { return gnss_published_; }

  double scale_cal_travelled_ = 0.0;
  std::vector<double> scale_cal_samples_;
  std::vector<double> scale_cal_wheels_;
  bool scale_cal_done_ = false;
  bool ml_applied_ = false;
  double prev_u_ = 0.0;
  bool have_prev_u_ = false;
  double gnss_scale_ref_ = 0.0;
  double s_map_offset_ = 0.0;
  bool s_map_offset_valid_ = false;
  double gnss_prev_e_ = 0.0, gnss_prev_n_ = 0.0;
  double gnss_prev_t_ = -1e9;
  double aux_psi_ = 0.0;  ///< integrated heading from the antenna baseline
};

}  // namespace tram
