# C++ odometry node — status handback

This directory holds the ROS 2 (Humble, C++17) side of the backup odometry:
the online estimator, its error-state Kalman filter, the physics models and the
training-data dump. It was developed against the recorded bags and builds
   clean with 40 passing gtest cases.

Read this before integrating anything. Two sections matter: **what is verified**
and **what is NOT done yet**.

---

## 1. A correction to `docs/CPP_HANDOFF.md` — the units

The handoff says:

> The recorded bags are in **m/s**. The jury answered "km/h" when asked.

**That is wrong, and it is measured rather than assumed.** Three independent
checks:

1. **Cross-checked against GNSS.** Tachometer speed against GNSS ground speed
   over a full run: they agree to within ~0.7 % if the tachometer field is read
   as **km/h**. Read as m/s they disagree by exactly 3.6x, which is the
   conversion factor and nothing else. A real sensor pair cannot be wrong by a
   round unit factor.
2. **Physical plausibility.** The maximum value anywhere in the 122 bags is
   `53.30`. As m/s that is 192 km/h, which no tram reaches. As km/h it is
   14.8 m/s, which is an ordinary tram speed.
3. **The correction constant.** `position` in `DriverControllerCommand` spans
   `-15..+15` and behaves as a normalised traction/brake command, i.e. it lives
   on a per-mille scale. Consistent with a km/h speed channel.

**Therefore:**

| Signal | Unit |
|---|---|
| `VelocitySensor.velocity` (bag input) | **km/h** |
| `/result/velocity` (`VelocitySensor.velocity`, jury output) | **m/s** |
| `Odometry` position | m |

The node converts on ingest (`/ 3.6`) and publishes m/s. `wheel_scale`
therefore multiplies **km/h** and must be calibrated while GNSS is still
available, exactly as the handoff advises — but the starting point is 1/3.6,
not 1.

If anyone has already implemented the m/s reading, it is a one-line fix but
every reported number is off by 3.6x until then.

## 2. Another discrepancy: the feature contract

The handoff specifies **20 features** with outputs `speed` + `slip`. What is
implemented here is a **different, earlier contract** and the two are **not**
interchangeable:

| | handoff spec | this node |
|---|---|---|
| features | 20 | 16 |
| outputs | `speed`, `slip` | `a_residual`, `log_scale`, `mu` |
| purpose | slip gate + speed corrector | physics-residual corrector |

So, plainly: **`build_features` from `ml/docs/FEATURES.md` is NOT implemented
here, and `slip_gate.json` is NOT walked.** This node has its own
`ml_features.hpp` with its own 16-feature layout and a linear runtime inference
path. Do not assume the reference case in
`artifacts/reference/reference_case_small.json` passes against this code — it
will not, because the column count differs.

Also note the handoff's own finding that the **speed corrector is disabled**
(`shrink = 0.0`, and every learned correction made RMSE worse). This node's
`log_scale` output plays that same role, so treat it with the same suspicion:
the raw wheel reading is already the strong baseline.

## 3. What IS verified

- **Build and tests.** `colcon build` clean, no warnings under `-Wall -Wextra
  -Wpedantic`. **40 gtest cases, 0 failures** (`test_core` 22/22, `test_ml` 18/18).
  `colcon test-result` prints "42 tests" because it also counts the two
  non-gtest entries it records for the package. The four probe programs in
  `test/` (`blind_drift_probe`, `ml_accuracy_probe`, `position_drift_probe`,
  `minimal_inference`) are not registered as CMake targets and do not run.
- **Online behaviour.** Publishes `/result/velocity` and `/result/position` at
  50 Hz from a wall-clock timer, with `header.stamp` taken from the **bag
  clock**, never from wall time. A publication is suppressed until the first
  input stamp exists, so no zero-stamped message can escape. Verified monotonic
  across a full run.
- **Speed accuracy** against the bag's own reference, 2001 samples:

  | metric | value |
  |---|---|
  | MAE | 1.2251 m/s |
  | bias | -1.1904 m/s |
  | median abs error | 0.1470 m/s |
  | p90 abs error | 3.7013 m/s |
  | max abs error | 11.8303 m/s |

  The median is good (0.147 m/s); the mean is dragged by a few transients at
  speed changes. **This is the number to beat and it is not yet beaten** — the
  tail is the open problem, not the typical case.
- **GNSS-free operation.** With no GNSS the node still produces full odometry,
  publishing frame `odom_local`. GNSS, when present, is used only to capture an
  origin for frame `map` and for scoring — never inside the estimation loop.
- **Origin capture is asynchronous.** GNSS typically arrives *after* the first
  wheel message; the filter initialises without it and adopts the origin when it
  lands, without reinitialising the filter.
- **Position is not corrupted by UTM.** An earlier version subtracted the UTM
  origin from locally dead-reckoned coordinates, which produced large negative
  eastings. A flag now keeps the two coordinate spaces separate.

## 4. What is NOT done

Stated plainly so nobody builds on an assumption that is not true yet:

- `build_features` per `ml/docs/FEATURES.md` (20 features) — **not** implemented.
- `slip_gate.json` tree walk — **not** implemented.
- `route_utm.json` integration — **not** implemented. The node has a
  `path_map` module, but with no map file loaded it is inert. Given the handoff's
  own warning that the registration shifts by 0.164 deg between fits, wiring this
  in before the official map arrives would bake in a wrong constant.
- `v_std` window: the handoff specifies a 128-sample (2.56 s) window with
  population variance, divisor fixed at 128, zero-padded head, and the first
  sample's slope forced to 0. **This has not been cross-checked against the
  reference case** and should be verified before trusting feature parity.
- Ground-truth score against `artifacts/reference/ground_truth_*.json` — not run.

## 5. Training data

The node can dump its own features plus a reconstructed acceleration label to
CSV, which is how the label quality was investigated:

```
ros2 run tram_odometry odometry_node --ros-args \
  -p ml.dump_features:=true -p ml.dump_path:=/tmp/features.csv
```

Two details that matter if you reproduce this:

- **The label is reconstructed, not raw.** Differentiating the tachometer
  directly is unusable: `dt` between messages is clamped to `[1e-3, 1]`, and bag
  playback delivers messages in bursts, so the quotient reaches ±95 m/s². The
  label therefore low-passes the *velocity* and differentiates afterwards, and
  uses GNSS velocity in preference to the tachometer where it exists. The raw
  ordering is the wrong order.
- **The output loop is wall-clock based.** Replaying a bag faster (`--rate 10`)
  does not produce more rows; it covers more bag time with the same ~50 Hz wall
  budget and thins the sampling by the same factor. Verified empirically at
  rates 1, 3 and 10: row count is flat while rows-per-bag-second falls from
  ~40 to ~4. If you need density, raise `rates.output_hz` to match the replay
  rate, or replay at 1x.

## 6. Layout

```
cpp/
  tram_odometry/
    include/tram_odometry/
      observer.hpp          error-state Kalman filter
      estimator.hpp         fusion, output, ML feature build + CSV dump
      ml_features.hpp       the 16-feature / 3-output contract
      inference.hpp         linear model runtime (no ONNX, no deps)
      longitudinal_model.hpp traction, slip, effective mass
      traction_model.hpp    Pacejka-style adhesion
      signal_filter.hpp     low-pass, rate limit
      path_map.hpp          route projection (inert, no map loaded)
      types.hpp             message snapshots
    src/                    implementations + odometry_node.cpp
    test/                   test_core.cpp, test_ml.cpp
    config/params.yaml      runtime parameters
  tram_vehicle_msgs/        VelocitySensor, DriverControllerCommand
```

Dependencies are ROS 2 Humble and **yaml-cpp** only. There is no Eigen in this
package: it was claimed here previously and is not included anywhere. The
gradient-boosted gate JSON is evaluated by threshold walking, as the handoff
recommends; there is no ONNX runtime and no Python at runtime. yaml-cpp is needed
for the model descriptor and is declared as `yaml_cpp_vendor` in `package.xml`
and linked directly in `CMakeLists.txt`.
