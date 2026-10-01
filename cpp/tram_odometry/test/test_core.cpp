// Unit tests for the model core. No ROS runtime needed:
//   colcon test --packages-select tram_odometry
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tram_odometry/estimator.hpp"
#include "tram_odometry/geo.hpp"
#include "tram_odometry/longitudinal_model.hpp"
#include "tram_odometry/observer.hpp"
#include "tram_odometry/path_map.hpp"
#include "tram_odometry/signal_filter.hpp"
#include "tram_odometry/traction_model.hpp"

using namespace tram;

// ------------------------------------------------------------------- geodesy

TEST(Geo, UtmRoundTrip) {
  const double lat = 55.751244;  // Moscow
  const double lon = 37.618423;
  const UtmPoint p = wgs84_to_utm(lat, lon, 0);
  ASSERT_TRUE(p.valid);
  EXPECT_EQ(p.zone, 37);
  // Easting of a point in the middle of the zone is close to 500 km.
  EXPECT_NEAR(p.easting, 500000.0, 250000.0);

  double lat_back = 0.0, lon_back = 0.0;
  utm_to_wgs84(p, lat_back, lon_back);
  EXPECT_NEAR(lat_back, lat, 1e-7);
  EXPECT_NEAR(lon_back, lon, 1e-7);
}

TEST(Geo, UtmZoneSelection) {
  EXPECT_EQ(utm_zone_for_lon(37.6), 37);
  EXPECT_EQ(utm_zone_for_lon(-179.9), 1);
  EXPECT_EQ(utm_zone_for_lon(179.9), 60);
  EXPECT_FALSE(wgs84_to_utm(95.0, 10.0, 0).valid);  // out of the UTM band
}

TEST(Geo, ConvergenceIsSmallButNonZero) {
  // 1.3 deg of meridian convergence on this route: the ENU helper must rotate
  // the UTM delta, otherwise a 5 km run would drift ~100 m.
  const double gamma = utm_convergence_rad(55.75, 40.0, 37);
  EXPECT_LT(std::fabs(gamma), 0.05);
  double x = 0.0, y = 0.0;
  utm_delta_to_enu(1000.0, 0.0, 55.75, 40.0, 37, x, y);
  EXPECT_NEAR(std::hypot(x, y), 1000.0, 1e-6);
}

// ------------------------------------------------------------------- filters

TEST(Filters, HampelRejectsSpike) {
  HampelFilter f(7, 3.0);
  for (int i = 0; i < 20; ++i) EXPECT_FALSE(f.push(10.0 + 0.01 * i));
  EXPECT_TRUE(f.push(90.0));  // spike
  EXPECT_NEAR(f.lastAccepted(), 10.19, 0.05);
  EXPECT_FALSE(f.push(10.2));
}

TEST(Filters, HampelRejectsNonFinite) {
  HampelFilter f(5, 3.0);
  f.push(1.0);
  EXPECT_TRUE(f.push(std::nan("")));
  EXPECT_TRUE(f.push(INFINITY));
}

TEST(Filters, LowPassKeepsDcAndSmoothsStep) {
  LowPassFilter f(5.0, 0.02);
  double y = 0.0;
  for (int i = 0; i < 200; ++i) y = f.process(1.0, 0.02);
  EXPECT_NEAR(y, 1.0, 1e-3);

  LowPassFilter g(5.0, 0.02);
  double prev = 0.0;
  bool monotone = true;
  for (int i = 0; i < 500; ++i) {
    const double v = g.process(1.0, 0.02);
    if (v < prev - 1e-12) monotone = false;
    prev = v;
  }
  EXPECT_TRUE(monotone);
  EXPECT_GT(prev, 0.99);
}

TEST(Filters, FreezeDetector) {
  FreezeDetector fd(0.5);
  fd.push(0.0, 3.0);
  EXPECT_FALSE(fd.frozen(0.4));
  EXPECT_TRUE(fd.frozen(0.6));
  fd.push(0.7, 3.5);
  EXPECT_FALSE(fd.frozen(0.8));
}

TEST(Filters, SlewLimiterRespectsRate) {
  SlewLimiter s(1.0, 2.0);
  EXPECT_NEAR(s.step(100.0, 0.1), 0.1, 1e-9);
  EXPECT_NEAR(s.step(-100.0, 0.1), -0.1, 1e-9);
}

// ------------------------------------------------------------------ dynamics

TEST(Longitudinal, AdhesionLimitsAcceleration) {
  DynamicsParams dp;
  VehicleParams vp;
  AdhesionParams ap;
  LongitudinalModel m(dp, vp, ap);

  LongitudinalModel::Input in;
  in.v = 5.0;
  in.drive_force = 1e6;  // absurd demand
  in.mu = ap.mu_peak;
  const double f_limit = m.adhesionLimit(ap.mu_peak);
  EXPECT_NEAR(m.acceleration(in), f_limit / m.effectiveMass(), 1e-9);
  EXPECT_LT(m.acceleration(in), 1.0);  // a tram cannot pull more than ~1 m/s^2
}

TEST(Longitudinal, ResistanceGrowsWithSpeed) {
  DynamicsParams dp;
  VehicleParams vp;
  AdhesionParams ap;
  LongitudinalModel m(dp, vp, ap);
  const double r0 = m.resistance(1.0, 0.0, 0.0);
  const double r1 = m.resistance(15.0, 0.0, 0.0);
  EXPECT_GT(r1, r0);

  const double rg = m.resistance(0.0, 0.05, 0.0);
  EXPECT_GT(rg, r0);  // uphill costs more
}

TEST(Longitudinal, StandingStillDoesNotRollBackwards) {
  DynamicsParams dp;
  VehicleParams vp;
  AdhesionParams ap;
  LongitudinalModel m(dp, vp, ap);
  LongitudinalModel::Input in;
  in.v = 0.0;
  in.brake_force = 1000.0;
  EXPECT_NEAR(m.stepVelocity(in, 0.0, 0.02), 0.0, 1e-9);
}

TEST(Traction, ShapeIsNonLinearAndMonotone) {
  TractionParams tp;
  TractionModel t(tp);
  const double s0 = t.shapeFactor(0.0);
  const double s1 = t.shapeFactor(0.15);
  const double s2 = t.shapeFactor(0.5);
  const double s3 = t.shapeFactor(1.0);
  EXPECT_NEAR(s0, 0.0, 1e-9);
  EXPECT_NEAR(s3, 1.0, 1e-9);
  // More handle travel per unit of torque in the middle than at the ends:
  // that is exactly the non-linearity we must reproduce.
  EXPECT_LT(s2 - s1, s1 - s0);
  EXPECT_LT(s3 - s2, s2 - s1);
}

TEST(Traction, ConstantPowerLimitReducesForceAtSpeed) {
  TractionParams tp;
  TractionModel t(tp);
  EXPECT_NEAR(t.powerLimit(0.0), 1.0, 1e-6);
  EXPECT_LT(t.powerLimit(20.0), 1.0);
}

TEST(Traction, ActuatorLagSmoothsStep) {
  TractionParams tp;
  TractionModel t(tp);
  t.step(1.0, 0.0, 0.02);
  const double first = t.driveForce();
  EXPECT_LT(first, 0.5 * tp.max_tractive_effort_n);  // not instant
  for (int i = 0; i < 200; ++i) t.step(1.0, 0.0, 0.02);
  EXPECT_NEAR(t.driveForce(), tp.max_tractive_effort_n, 1e-3);
}

// ------------------------------------------------------------------ observer

TEST(Observer, ConvergesToWheelSpeed) {
  ObserverParams op;
  VehicleParams vp;
  Observer obs(op, vp);
  DynamicsParams dp;
  AdhesionParams ap;
  LongitudinalModel dyn(dp, vp, ap);
  TractionParams tp;
  TractionModel tr(tp);
  obs.setInitialVelocity(8.0);

  for (int i = 0; i < 2000; ++i) {
    tr.step(0.0, 8.0, 0.02);  // no traction, constant speed
    obs.predict(0.02, dyn, tr.driveForce(), tr.brakeForce(), 0.0, 0.0, ap.mu_peak);
    obs.updateWheels(8.0 / vp.wheel_radius_m, 8.0 / vp.wheel_radius_m, 1.0, 0.0);
  }
  EXPECT_NEAR(obs.v(), 8.0, 0.2);
  EXPECT_NEAR(obs.bScale(), 1.0, 0.05);
  EXPECT_GT(obs.s(), 100.0);
}

TEST(Observer, RecoversScaleError) {
  ObserverParams op;
  VehicleParams vp;
  Observer obs(op, vp);
  // The true radius is 2 % larger than the nominal one: the odometry
  // overestimates the travelled distance by 2 %.
  obs.calibrateScale(1000.0, 980.0);
  EXPECT_NEAR(obs.bScale(), 0.98, 1e-6);
  EXPECT_NEAR(obs.s(), 980.0, 1e-6);
}

TEST(Observer, LowTrustRejectsWheelUpdate) {
  ObserverParams op;
  op.r_wheel = 1e-6;
  op.r_wheel_degraded = 1e6;
  VehicleParams vp;
  Observer obs(op, vp);
  DynamicsParams dp;
  AdhesionParams ap;
  LongitudinalModel dyn(dp, vp, ap);
  TractionParams tp;
  TractionModel tr(tp);
  obs.setInitialVelocity(0.0);
  // Full slip: the wheels report 15 m/s while the model says the tram is stopped.
  for (int i = 0; i < 200; ++i) {
    tr.step(0.0, 0.0, 0.02);
    obs.predict(0.02, dyn, 0.0, 0.0, 0.0, 0.0, ap.mu_peak);
    obs.updateWheels(15.0 / vp.wheel_radius_m, 15.0 / vp.wheel_radius_m, 0.0, 1.0);
  }
  EXPECT_LT(obs.v(), 1.0);
}

TEST(Observer, InnovationGateRejectsOutlier) {
  ObserverParams op;
  VehicleParams vp;
  Observer obs(op, vp);
  obs.setInitialVelocity(5.0);
  for (int i = 0; i < 50; ++i) {
    obs.updateWheels(5.0 / vp.wheel_radius_m, 5.0 / vp.wheel_radius_m, 1.0, 0.0);
  }
  const double before = obs.v();
  // 30 m/s from the wheels while the tram does 5 m/s: must be gated out.
  const bool accepted = obs.updateWheels(30.0 / vp.wheel_radius_m, 30.0 / vp.wheel_radius_m, 1.0, 0.0);
  EXPECT_FALSE(accepted);
  EXPECT_NEAR(obs.v(), before, 1.0);
}

// ------------------------------------------------------------------ path map

TEST(PathMap, StraightLineProjection) {
  PathMap m;
  for (int i = 0; i <= 100; ++i) {
    PathPoint p;
    p.x = i * 10.0;
    p.y = 0.0;
    p.z = 0.0;
    m.points().push_back(p);
  }
  m.finalise();
  EXPECT_NEAR(m.totalLength(), 1000.0, 1e-6);

  PathPoint out;
  double along = 0.0, cross = 0.0;
  ASSERT_TRUE(m.project(505.0, 3.0, out, along, cross));
  EXPECT_NEAR(along, 505.0, 1.0);
  EXPECT_NEAR(cross, 3.0, 0.1);

  PathPoint at;
  ASSERT_TRUE(m.pointAt(250.0, at));
  EXPECT_NEAR(at.x, 250.0, 1.0);
}

TEST(PathMap, RejectsProjectionTooFarAway) {
  PathMap m;
  for (int i = 0; i <= 10; ++i) {
    PathPoint p;
    p.x = i * 10.0;
    m.points().push_back(p);
  }
  m.finalise();
  PathPoint out;
  double along = 0.0, cross = 0.0;
  EXPECT_FALSE(m.project(50.0, 500.0, out, along, cross, 25.0));
}

// ----------------------------------------------------------------- estimator

TEST(Estimator, EndToEndSyntheticRun) {
  Params p;
  p.rates.output_hz = 50.0;
  p.frame.origin_lat = 55.75;
  p.frame.origin_lon = 37.61;
  p.frame.origin_alt = 100.0;
  p.vehicle.mass_kg = 38000.0;
  p.path_map.enable = false;
  Estimator est(p);

  // Constant 8 m/s cruise, wheels with a 3 % scale error, GNSS for the first 2 s.
  const double v_true = 8.0;
  const double wheel_kmh = v_true * 3.6 * 1.03;
  double t = 100.0;
  est.onGnssFix(t, 55.75, 37.61, 100.0, 12, 1, 0.01);
  est.onGnssVel(t, v_true, 0.0, 0.0);
  est.onDriver(t, 0.0);

  bool ok = false;
  for (int i = 0; i < 1000; ++i) {
    t += 0.02;
    est.onWheelFront(t, wheel_kmh);
    est.onWheelRear(t, wheel_kmh);
    est.onDriver(t, 0.0);
    if (i < 100) {  // GNSS available only at the beginning
      est.onGnssFix(t, 55.75, 37.61, 100.0, 12, 1, 0.01);
      est.onGnssVel(t, v_true, 0.0, 0.0);
    }
    ok = est.step(t);
  }
  ASSERT_TRUE(ok);
  const Estimate e = est.estimate();
  EXPECT_NEAR(e.v, v_true, 0.2);
  EXPECT_NEAR(e.b_scale, 1.0 / 1.03, 0.02);
  // 20 s at 8 m/s, the distance must be right despite the raw scale error.
  EXPECT_NEAR(e.s, 20.0 * v_true, 5.0);
  EXPECT_FALSE(e.model_only);
  EXPECT_GT(e.trust, 0.5);
}

TEST(Estimator, SurvivesDropoutAndNaN) {
  Params p;
  p.path_map.enable = false;
  Estimator est(p);
  double t = 10.0;
  est.onGnssFix(t, 55.75, 37.61, 100.0, 12, 1, 0.01);
  est.onGnssVel(t, 5.0, 0.0, 0.0);
  for (int i = 0; i < 200; ++i) {
    t += 0.02;
    est.onWheelFront(t, 18.0);
    est.onWheelRear(t, 18.0);
    EXPECT_TRUE(est.step(t));
  }
  // Now the front sensor dies and the rear one returns garbage.
  for (int i = 0; i < 200; ++i) {
    t += 0.02;
    est.onWheelRear(t, std::nan(""));
    EXPECT_TRUE(est.step(t));
  }
  const Estimate e = est.estimate();
  EXPECT_TRUE(std::isfinite(e.v));
  EXPECT_LT(e.trust, 0.6);          // confidence must drop
  EXPECT_LT(std::fabs(e.s), 1e6);   // no drift blow-up
}
