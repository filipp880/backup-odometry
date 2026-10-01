// Preprocessing primitives: outlier rejection, low-pass, slew limiting and
// stuck-sensor detection. All of them are fixed-size and allocation free.
#pragma once

#include <cmath>
#include <cstddef>

namespace tram {

/// Median/Hampel outlier rejection over a sliding window.
/// push() returns true when the new sample is rejected as an outlier.
class HampelFilter {
 public:
  HampelFilter(int window, double sigma);
  bool push(double x);
  double value() const { return median_; }
  double lastAccepted() const { return last_; }
  void reset();

 private:
  static constexpr int kMax = 32;
  int window_;
  double sigma_;
  double buf_[kMax];
  double scratch_[kMax];
  int count_ = 0;
  int head_ = 0;
  double median_ = 0.0;
  double last_ = 0.0;
  bool has_value_ = false;
};

/// 4th-order Butterworth low-pass (two cascaded RBJ biquads).
class LowPassFilter {
 public:
  LowPassFilter(double cutoff_hz, double nominal_dt);
  double process(double x, double dt);
  void reset();

 private:
  void design(double cutoff_hz, double dt);
  struct Biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    double z1 = 0, z2 = 0;
    double step(double x) {
      const double y = b0 * x + z1;
      z1 = b1 * x - a1 * y + z2;
      z2 = b2 * x - a2 * y;
      return y;
    }
    void reset() {
      z1 = z2 = 0;
    }
  };
  Biquad s1_, s2_;
  double cutoff_ = 10.0;
  double nominal_dt_ = 0.02;
  double designed_dt_ = -1.0;
  double designed_cutoff_ = -1.0;
};

/// First-order lag: models actuator/plant inertia with time constant tau.
class FirstOrderLag {
 public:
  explicit FirstOrderLag(double tau_s) : tau_(tau_s) {}
  double step(double target, double dt) {
    if (tau_ <= 1e-6) {
      y_ = target;
      return y_;
    }
    const double a = dt / (tau_ + dt);
    y_ += a * (target - y_);
    return y_;
  }
  double value() const { return y_; }
  void reset(double v = 0.0) { y_ = v; }

 private:
  double tau_;
  double y_ = 0.0;
};

/// Rate limiter used to keep accelerations inside physical limits.
class SlewLimiter {
 public:
  SlewLimiter(double rate_up, double rate_down) : up_(rate_up), down_(rate_down) {}
  double step(double value, double dt) {
    const double max_step = (value > prev_ ? up_ : down_) * dt;
    double d = value - prev_;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    prev_ += d;
    return prev_;
  }
  double value() const { return prev_; }
  void reset(double v = 0.0) { prev_ = v; }

 private:
  double up_, down_, prev_ = 0.0;
};

/// Detects a frozen sensor: value constant for longer than the timeout.
class FreezeDetector {
 public:
  explicit FreezeDetector(double timeout_s) : timeout_(timeout_s) {}
  void push(double t, double value) {
    if (!has_prev_) {
      has_prev_ = true;
      last_value_ = value;
      last_change_t_ = t;
    } else if (std::fabs(value - last_value_) > eps_) {
      last_value_ = value;
      last_change_t_ = t;
    }
  }
  bool frozen(double t) const { return has_prev_ && (t - last_change_t_) > timeout_; }
  double age(double t) const { return has_prev_ ? t - last_change_t_ : 0.0; }
  void reset() {
    has_prev_ = false;
    last_value_ = 0;
    last_change_t_ = 0;
  }

 private:
  double timeout_;
  double eps_ = 1e-4;
  bool has_prev_ = false;
  double last_value_ = 0.0;
  double last_change_t_ = 0.0;
};

}  // namespace tram
