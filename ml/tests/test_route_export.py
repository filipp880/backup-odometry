"""The exported route and reference trajectory must be mutually consistent.

A wrong transform here is invisible in the file itself -- the numbers look
perfectly reasonable -- and only shows up as a position estimate that is
thousands of kilometres wrong.  The cross-track check below is the guard: it
compares the reference trajectory against the route it claims to follow.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from odom_ml.export import to_array
from odom_ml.position.projection import MapProjection

ROUTE = Path("artifacts/route/route_utm.json")
TRUTH_DIR = Path("artifacts/reference")


def _load(path: Path) -> dict:
    if not path.exists():
        pytest.skip(f"{path} not generated; run ml/scripts/23_export_route.py")
    return json.loads(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def route() -> dict:
    return _load(ROUTE)


@pytest.fixture(scope="module")
def route_arrays(route) -> dict[str, np.ndarray]:
    p = np.asarray(route["points"], dtype=np.float64)
    names = route["columns"]
    return {n: p[:, i] for i, n in enumerate(names)}


def test_route_covers_the_whole_alignment(route_arrays):
    assert route_arrays["s"].size == 4710
    assert route_arrays["s"][0] == 0.0
    assert route_arrays["s"][-1] == pytest.approx(4708.3, abs=0.5)
    assert np.all(np.diff(route_arrays["s"]) > 0), "arclength must increase"


def test_northing_has_no_false_northing(route_arrays):
    """UTM northings here are ~6.19e6, not ~16.19e6.

    Mixing the two conventions is a 10 000 km error, and the artefact has to say
    which one it uses without the reader having to infer it.
    """
    n = route_arrays["y"]
    assert 6.0e6 < n.mean() < 7.0e6, f"unexpected northing range {n.min()}..{n.max()}"
    assert "false_northing" in json.loads(ROUTE.read_text(encoding="utf-8"))["conventions"]


def test_tangents_and_normals_are_orthonormal_unit_vectors(route_arrays):
    t = np.column_stack([route_arrays["t_x"], route_arrays["t_y"]])
    nvec = np.column_stack([route_arrays["n_x"], route_arrays["n_y"]])
    np.testing.assert_allclose(np.linalg.norm(t, axis=1), 1.0, atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(nvec, axis=1), 1.0, atol=1e-9)
    # left normal: rotating the tangent by +90 deg
    np.testing.assert_allclose(nvec[:, 0], -t[:, 1], atol=1e-9)
    np.testing.assert_allclose(nvec[:, 1], t[:, 0], atol=1e-9)


def test_route_reprojects_onto_itself_in_utm(route_arrays):
    """Feeding the exported UTM back through the projection must recover s."""
    mp = MapProjection.load()
    proj = mp.project_utm(route_arrays["x"], route_arrays["y"])
    assert np.abs(proj.s - route_arrays["s"]).max() < 1.0
    assert np.abs(proj.lateral).max() < 1.0


def test_heading_is_unwrapped_and_consistent_with_the_tangent(route_arrays):
    """heading must be continuous along the route, not wrapped per vertex.

    Tolerances are set from the measured round-trip error of the 9-significant
    digit JSON, not chosen to make the test pass: storing a derived value and
    recomputing its trig identity independently disagrees in the last digits.
    """
    deg = route_arrays["heading_deg"]
    rad = route_arrays["heading_rad"]
    np.testing.assert_allclose(np.degrees(rad), deg, atol=1e-6)
    assert np.abs(np.diff(deg)).max() < 90.0, "heading wraps somewhere along the route"
    np.testing.assert_allclose(np.cos(rad), route_arrays["t_x"], atol=1e-8)
    np.testing.assert_allclose(np.sin(rad), route_arrays["t_y"], atol=1e-8)


def test_grade_matches_the_height_profile(route_arrays):
    """grade is dz/ds and grade_rad its angle; the gradient must integrate back."""
    s_, z_ = route_arrays["s"], route_arrays["z"]
    np.testing.assert_allclose(
        route_arrays["grade_rad"], np.arctan(route_arrays["grade"]), atol=1e-9
    )
    np.testing.assert_allclose(route_arrays["grade"], np.gradient(z_, s_), atol=1e-6)
    trap = np.concatenate(
        [[0.0], np.cumsum(0.5 * (route_arrays["grade"][1:] + route_arrays["grade"][:-1]) * np.diff(s_))]
    )
    assert np.abs(z_[0] + trap - z_).max() < 1.0, "grade does not integrate back to z"


def test_curvature_is_the_derivative_of_heading(route_arrays):
    """curvature is a second derivative, so only an absolute bound is meaningful.

    It passes through zero along a straight section, where a relative tolerance
    would be comparing noise with noise.
    """
    s_ = route_arrays["s"]
    np.testing.assert_allclose(
        route_arrays["curvature"], np.gradient(route_arrays["heading_rad"], s_), atol=1e-6
    )
    assert (np.abs(route_arrays["curvature"]) < 1e-12).any(), (
        "no straight section, so the infinite-radius case is never exercised"
    )


@pytest.mark.parametrize("path", sorted(TRUTH_DIR.glob("ground_truth_*.json")))
def test_ground_truth_actually_follows_the_route(path):
    """The guard against a double transform.

    A reference trajectory produced by pushing the GNSS through the registration
    as well as through the geodetic conversion lands thousands of kilometres away
    and still serialises perfectly.  Its cross-track against the route does not.
    """
    doc = json.loads(path.read_text(encoding="utf-8"))
    rows = doc["rows"]
    ct = to_array([r[5] for r in rows])
    on = np.array([r[6] for r in rows], dtype=bool)
    assert on.any(), "no on-route rows at all"

    assert np.abs(ct[on]).max() < 50.0, (
        f"on-route rows are up to {np.abs(ct[on]).max():.0f} m from the corridor"
    )
    assert np.median(np.abs(ct[on])) < 10.0
    # off-route rows are the depot and should genuinely be far away
    if (~on).any():
        assert np.abs(ct[~on]).min() > 15.0, "rows marked off-route are on the corridor"

    s = to_array([r[4] for r in rows])
    assert s[on].max() - s[on].min() > 1000.0, "run does not traverse the route"
