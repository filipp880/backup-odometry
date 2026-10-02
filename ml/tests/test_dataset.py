"""The dataset is the source of truth, so it is itself under test.

These guard the property everything else depends on: that the code can see the
vendored bags, that the cache is *derived* from them rather than merely
coexisting, and that the recorded telemetry has the physical structure the rest
of the code assumes.

Every run here inflates compressed caches, so the tests that sweep the whole
fleet are marked ``slow`` and excluded from the default run.  They read only the
arrays they need -- a run holds ~34 and each one touched costs a zlib inflate.
Run them with ``pytest ml/tests -m slow``.
"""

from __future__ import annotations

import numpy as np
import pytest

from conftest import (
    BASELINE_TOL_M,
    NOMINAL_BASELINE_M,
    antenna_separation,
    bag_topics,
    reference_bags,
    require_cache,
    require_dataset,
)

from odom_ml import config as C
from odom_ml.data.bag import CUSTOM_MSGS, build_typestore
from odom_ml.data.build import HZ, build_one, load_keys, manifest
from odom_ml.geo import enu_to_utm

pytestmark = pytest.mark.dataset

EXPECTED_TOPICS = set(C.ALL_TOPICS)
UTM_ZONE = 37


# --- cheap: structure only, no per-sample data ------------------------------


def test_dataset_has_the_documented_number_of_runs(dataset_bags):
    assert len(dataset_bags) == 122


def test_every_bag_is_a_readable_rosbag2_directory(dataset_bags):
    for bid in dataset_bags:
        d = C.BAGS_DIR / bid
        assert (d / "metadata.yaml").is_file(), bid
        assert list(d.glob("*.db3")), f"{bid} has no sqlite3 payload"


def test_message_definitions_are_present_and_registered():
    require_dataset()
    stems = {p.stem for p in C.MSG_DIR.glob("*.msg")}
    assert stems == set(CUSTOM_MSGS)
    build_typestore()  # raises if the custom types cannot be registered


def test_every_run_carries_the_same_seven_topics(dataset_bags):
    for bid in dataset_bags:
        assert bag_topics(bid) == EXPECTED_TOPICS, f"{bid} has a different topic set"


# --- slow: sweep the fleet ---------------------------------------------------


@pytest.mark.slow
def test_both_gnss_receivers_are_usable():
    """Runs must carry both antennas to serve as their own reference.

    39 of the 122 runs have no usable GNSS and the two antennas are not present
    in exactly the same set, so the guarantee is made over the runs that do have
    them rather than over the whole dataset.
    """
    master = set(reference_bags("master"))
    rover = set(reference_bags("rover"))
    assert len(master) >= 50, f"only {len(master)} runs with a master fix"
    assert len(rover) >= 50, f"only {len(rover)} runs with a rover fix"
    assert len(master & rover) >= 50, "too few runs have both antennas"


@pytest.mark.slow
def test_antenna_separation_matches_the_vehicle_baseline():
    """Master and rover are 12.436 m apart on the same rigid body.

    Neither is used to derive the other, so agreement is a real check on the
    whole geodetic chain: two independent fixes, converted through ENU and UTM by
    different code paths, must land a known distance apart.
    """
    seps = [
        antenna_separation(bid)
        for bid in sorted(set(reference_bags("master")) & set(reference_bags("rover")))
    ]
    assert len(seps) >= 20
    med = float(np.median(seps))
    assert abs(med - NOMINAL_BASELINE_M) < BASELINE_TOL_M, (
        f"median antenna separation {med:.3f} m vs nominal {NOMINAL_BASELINE_M} m; "
        f"range {min(seps):.2f}..{max(seps):.2f} m"
    )


@pytest.mark.slow
def test_cache_is_derivable_from_the_vendored_dataset(tmp_path, monkeypatch):
    """Rebuilding a run from the vendored bag must reproduce the cached labels.

    This is what proves the pipeline reads the dataset rather than depending on
    some earlier state: the cache is regenerated from ``data/`` alone.
    """
    require_cache()
    bag_ids = [b["bag_id"] for b in manifest(HZ)["bags"]]
    if not bag_ids:
        pytest.skip("cache empty")
    bid = sorted(bag_ids)[0]
    reference = {
        k: v
        for k, v in np.load(
            C.CACHE_DIR / "labeled" / f"{bid}_{int(HZ)}hz.npz", allow_pickle=False
        ).items()
    }

    monkeypatch.setattr(C, "CACHE_DIR", tmp_path)
    import odom_ml.data.build as build_mod

    monkeypatch.setattr(build_mod, "_CACHE", tmp_path / "labeled")
    rebuilt = build_one(bid, HZ, force=True)

    with np.load(rebuilt, allow_pickle=False) as z:
        assert set(z.files) == set(reference), "rebuilt cache has a different schema"
        for k in sorted(reference):
            a, b = np.asarray(reference[k]), np.asarray(z[k])
            assert a.shape == b.shape, f"{k}: {a.shape} vs {b.shape}"
            if a.dtype.kind == "f":
                np.testing.assert_allclose(a, b, rtol=0, atol=1e-9, equal_nan=True, err_msg=k)
            else:
                np.testing.assert_array_equal(a, b, err_msg=k)


@pytest.mark.slow
def test_cached_runs_have_the_expected_schema_and_plausible_values(all_bags):
    require_cache()
    keys = ("t", "u", "v_front", "v_rear", "speed", "gx", "gy", "rgx", "rgy", "rspeed", "dt")
    for rec in all_bags:
        d = load_keys(rec["bag_id"], keys, HZ)
        n = d["t"].size
        assert n == rec["rows"]
        for k in keys:
            assert d[k].size == n, f"{rec['bag_id']}/{k}"
        assert np.allclose(d["dt"], 1.0 / HZ), rec["bag_id"]
        assert np.all(np.diff(d["t"]) > 0), f"{rec['bag_id']} time not increasing"
        assert np.nanmax(np.abs(d["u"])) <= 15.0, rec["bag_id"]
        assert np.nanmax(d["v_front"]) < 50.0, rec["bag_id"]
        for k in ("gx", "gy", "speed", "rgx", "rgy", "rspeed"):
            finite = np.isfinite(d[k])
            if finite.any():
                assert np.nanmax(np.abs(d[k][finite])) < 10_000.0, f"{rec['bag_id']}/{k}"


@pytest.mark.slow
def test_runs_without_a_fix_are_absent_not_garbage():
    """A run with no GNSS must be empty, not filled with a plausible-looking lie."""
    require_cache()
    empty = [
        b["bag_id"]
        for b in manifest(HZ)["bags"]
        if not np.isfinite(load_keys(b["bag_id"], ("gx",), HZ)["gx"]).any()
    ]
    assert empty, "expected some runs to have no usable GNSS"
    for bid in empty[:5]:
        d = load_keys(bid, ("gx", "speed"), HZ)
        assert not np.isfinite(d["gx"]).any()
        assert not np.isfinite(d["speed"]).any()


@pytest.mark.slow
def test_wheel_and_gnss_speeds_agree():
    """The wheel topic and the GNSS velocity topic must describe the same motion.

    A disagreement here would mean a unit or sign error in the extraction, which
    no downstream test could catch.
    """
    checked = 0
    for bid in reference_bags("master")[:40]:
        d = load_keys(bid, ("v_wheel_mean", "speed"), HZ)
        m = np.isfinite(d["v_wheel_mean"]) & np.isfinite(d["speed"]) & (d["speed"] > 1.0)
        if m.sum() < 500:
            continue
        w = float(np.median(d["v_wheel_mean"][m] / d["speed"][m]))
        assert 0.9 < w < 1.1, f"{bid}: wheel/GNSS speed ratio {w:.4f}"
        checked += 1
    assert checked >= 5


@pytest.mark.slow
def test_enu_conversion_reproduces_the_reported_velocity():
    """Differentiating the position track must give the reported GNSS speed.

    ``gx/gy`` and ``speed`` come from different topics (fix vs vel), so this ties
    the geodetic projection to an independent measurement in the same bag.
    """
    checked = 0
    for bid in reference_bags("master"):
        d = load_keys(bid, ("gx", "gy", "speed", "dt"), HZ)
        if d["t" if "t" in d else "gx"].size < 2000:
            continue
        ok = np.isfinite(d["gx"]) & np.isfinite(d["gy"])
        if ok.sum() < 2000:
            continue
        dt = float(d["dt"][0])
        gx, gy = d["gx"][ok], d["gy"][ok]
        # central difference over 1 s, then compare where the vehicle is moving
        w = int(round(1.0 / dt)) | 1
        k = np.ones(w) / w
        sx, sy = np.convolve(gx, k, "same"), np.convolve(gy, k, "same")
        v = np.hypot(np.gradient(sx, dt), np.gradient(sy, dt))
        m = (d["speed"][ok] > 3.0) & np.isfinite(v)
        if m.sum() < 500:
            continue
        ratio = float(np.median(v[m] / d["speed"][ok][m]))
        assert 0.95 < ratio < 1.05, f"{bid}: differentiated/speed ratio {ratio:.4f}"
        checked += 1
        if checked >= 8:
            break
    assert checked >= 3


@pytest.mark.slow
def test_utm_conversion_preserves_distance_on_a_real_track():
    """ENU -> UTM must not distort lengths.

    The jury frame is UTM grid axes, so an anisotropic error here would show up
    as a position error that looks like a wheel-scale error.  The ENU track is the
    reference: it is metric by construction at the origin, while the UTM grid is
    only near-conformal, so comparing the two over kilometres catches a mistake in
    the projection that a round-trip test would hide.
    """
    checked = 0
    for bid in reference_bags("master"):
        d = load_keys(bid, ("gx", "gy", "gnss_lat0", "gnss_lon0"), HZ)
        ok = np.isfinite(d["gx"]) & np.isfinite(d["gy"])
        if ok.sum() < 2000:
            continue
        enu = np.column_stack([d["gx"][ok], d["gy"][ok], np.zeros(int(ok.sum()))])
        e, n, _ = enu_to_utm(
            enu, float(d["gnss_lat0"]), float(d["gnss_lon0"]), 0.0, zone=UTM_ZONE
        )
        len_utm = np.hypot(np.diff(e), np.diff(n))
        len_enu = np.hypot(np.diff(d["gx"][ok]), np.diff(d["gy"][ok]))
        good = len_enu > 1.0
        if good.sum() < 500:
            continue
        ratio = float(np.median(len_utm[good] / len_enu[good]))
        assert 0.999 < ratio < 1.001, f"{bid}: UTM/ENU length ratio {ratio:.6f}"
        checked += 1
        if checked >= 5:
            break
    assert checked >= 2
