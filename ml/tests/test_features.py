"""The feature contract must stay true.

Two things are easy to break silently and expensive to discover late: the
reference vectors drifting away from ``build_features``, and the written
specification drifting away from the code.  Both are checked here against the
dataset artefacts rather than against numbers copied into the test.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import pytest

from odom_ml.data.build import HZ, load_labeled
from odom_ml.data.features import (
    MAX_ACCEL,
    MAX_NOTIONAL_RATE,
    build_features,
    feature_names,
    iir_coefficients,
    rolling_std_window,
)
from odom_ml.models.export import predict_json, verify_export
from odom_ml.models.speed_slip import load_model

DOC = Path(__file__).resolve().parents[1] / "docs" / "FEATURES.md"
REFERENCE_DIR = Path("artifacts/reference")


def _load_reference(name: str) -> dict:
    path = REFERENCE_DIR / name
    if not path.exists():
        pytest.skip(f"{path} not generated; run ml/scripts/22_export_reference_cases.py")
    return json.loads(path.read_text(encoding="utf-8"))


def _to_array(values) -> np.ndarray:
    return np.array([np.nan if v is None else float(v) for v in values], dtype=np.float64)


@pytest.fixture(scope="module")
def real_run() -> dict:
    return load_labeled("30618_01f73500", HZ)


# --- the contract itself ----------------------------------------------------


def test_output_vector_is_twenty_named_floats():
    x, spec = build_features(np.zeros(10), np.zeros(10), np.zeros(10))
    assert x.shape == (10, len(feature_names()))
    assert x.dtype == np.float32
    assert spec.names == feature_names()


def test_features_are_causal(real_run):
    """Corrupting the future must not move the past by a single bit."""
    n = 4000
    u = real_run["u"][:n].copy()
    vf = real_run["v_front"][:n].copy()
    vr = real_run["v_rear"][:n].copy()
    a, _ = build_features(u, vf, vr)
    rng = np.random.default_rng(0)
    half = n // 2
    u[half:] = rng.uniform(-15, 15, n - half)
    vf[half:] = rng.uniform(0, 14, n - half)
    vr[half:] = rng.uniform(0, 14, n - half)
    b, _ = build_features(u, vf, vr)
    assert np.array_equal(a[:half], b[:half])


def test_features_are_always_finite_despite_gaps_in_the_topics(real_run):
    n = 6000
    u, vf, vr = real_run["u"][:n], real_run["v_front"][:n], real_run["v_rear"][:n]
    assert not np.isfinite(vf).all(), "expected gaps in the wheel topic"
    x, _ = build_features(u, vf, vr)
    assert np.isfinite(x).all()


def test_v_wheel_falls_back_to_the_valid_bogie(real_run):
    vf = real_run["v_front"][:2000].astype(np.float64).copy()
    vr = real_run["v_rear"][:2000].astype(np.float64).copy()
    vf[100:110] = np.nan
    x, _ = build_features(np.zeros(2000), vf, vr)
    col = x[:, feature_names().index("v_wheel")]
    np.testing.assert_allclose(col[100:110], vr[100:110], rtol=0, atol=1e-6)
    # a column with no valid reading anywhere falls back to zero
    vr[:] = np.nan
    y, _ = build_features(np.zeros(2000), np.full(2000, np.nan), vr)
    assert np.all(y[:, feature_names().index("v_wheel")] == 0.0)


def test_acceleration_is_clamped_to_the_physical_envelope(real_run):
    """A dropout recovery must not leak hundreds of m/s^2 into the features."""
    n = 3000
    v = np.zeros(n)
    v[: n // 2] = 5.0
    v[n // 2 :] = 0.0  # one-sample collapse of 5 m/s = 250 m/s^2
    x, _ = build_features(np.zeros(n), v, v)
    col = x[:, feature_names().index("a_wheel")]
    assert np.abs(col).max() <= MAX_ACCEL + 1e-6


def test_gap_counters_count_samples_since_the_last_valid_reading():
    n = 50
    vf = np.full(n, 3.0)
    vf[10:15] = np.nan
    x, _ = build_features(np.zeros(n), vf, np.full(n, 3.0))
    gap = x[:, feature_names().index("gap_front")]
    assert gap[9] == 0.0
    np.testing.assert_allclose(gap[10:15], [1, 2, 3, 4, 5])
    assert gap[15] == 0.0


# --- the documented numbers -------------------------------------------------


def test_rolling_std_window_is_the_documented_128():
    """The nominal window is 100 samples; the effective one is 128.

    If this ever changes, the written spec and every C++ port must change with
    it, so the value is pinned rather than derived.
    """
    win = rolling_std_window()
    assert win["requested_samples"] == 100
    assert win["effective_samples"] == 128
    assert win["effective_s"] == pytest.approx(2.56)


@pytest.mark.skipif(not DOC.exists(), reason="docs/FEATURES.md missing")
def test_documented_iir_coefficients_match_the_code():
    doc = DOC.read_text(encoding="utf-8")
    for name, coef in iir_coefficients().items():
        assert f"{coef['a']:.9f}" in doc, f"{name}: a not documented"
        assert f"{coef['b']:.9f}" in doc, f"{name}: b not documented"


@pytest.mark.skipif(not DOC.exists(), reason="docs/FEATURES.md missing")
def test_documented_output_table_matches_feature_names():
    """The table of outputs must list all 20 features, in the emitted order."""
    doc = DOC.read_text(encoding="utf-8")
    rows = re.findall(r"^\|\s*(\d+)\s*\|\s*`([a-z_0-9]+)`\s*\|", doc, re.M)
    listed = [name for _, name in sorted(rows, key=lambda r: int(r[0]))]
    assert listed == feature_names()


@pytest.mark.skipif(not DOC.exists(), reason="docs/FEATURES.md missing")
def test_documented_clamps_match_the_code():
    doc = DOC.read_text(encoding="utf-8")
    assert f"{MAX_ACCEL:.1f}" in doc
    assert f"{MAX_NOTIONAL_RATE:.0f}" in doc


# --- the exported artefacts -------------------------------------------------


@pytest.mark.parametrize("name", ["reference_case_small.json", "reference_case_full.json"])
def test_reference_case_reproduces_from_its_own_inputs(name):
    case = _load_reference(name)
    gate = json.loads(Path("artifacts/models/speed_slip/slip_gate.json").read_text(encoding="utf-8"))
    inp = case["inputs"]
    x, _ = build_features(
        _to_array(inp["u"]), _to_array(inp["v_front"]), _to_array(inp["v_rear"]), hz=HZ
    )
    stored = np.asarray(case["expected"]["features"], dtype=np.float32)
    assert np.abs(x - stored).max() <= case["tolerance"]["features_abs"]

    p = predict_json(gate, x.astype(np.float64))
    stored_p = _to_array(case["expected"]["slip_probability"])
    assert np.abs(p - stored_p).max() <= case["tolerance"]["probability_abs"]
    fired = (p >= case["expected"]["slip_gate_threshold"]).astype(int)
    assert np.array_equal(fired, _to_array(case["expected"]["slip_gate_fired"]))


@pytest.mark.parametrize("name", ["reference_case_small.json", "reference_case_full.json"])
def test_reference_case_contains_gaps_for_the_fallback_path(name):
    """If the case had no NaN, a port would never exercise the fallback rules."""
    case = _load_reference(name)
    n_null = sum(v is None for k in ("u", "v_front", "v_rear") for v in case["inputs"][k])
    assert n_null > 0, "reference case contains no gaps, fallback path untested"


@pytest.mark.skipif(
    not Path("artifacts/models/speed_slip/speed_slip.joblib").exists(),
    reason="joblib model not exported (pickles are gitignored by design)",
)
def test_exported_gate_still_reproduces_sklearn(real_run):
    model = load_model("artifacts/models/speed_slip/speed_slip.joblib")
    doc = json.loads(Path("artifacts/models/speed_slip/slip_gate.json").read_text(encoding="utf-8"))
    n = 4000
    x, _ = build_features(
        real_run["u"][:n], real_run["v_front"][:n], real_run["v_rear"][:n], hz=HZ
    )
    report = verify_export(model.slip_model, doc, x, tol=1e-12)
    assert report["ok"], f"max diff {report['max_abs_diff']:.3e}"
