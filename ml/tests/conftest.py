"""Shared fixtures.

Every accuracy or geometry assertion in this suite is checked against the
dataset itself: the GNSS fixes recorded in the bags, the GNSS velocity topic, the
12.436 m antenna baseline, and the route.  Nothing is validated against a number
the code produced earlier in the same run.

The pure-algebra unit tests (does projecting invert, does the output frame
subtract the origin) stay synthetic on purpose -- they test arithmetic, not
accuracy, and a hand-built route is a much sharper probe for them.
"""

from __future__ import annotations

import re
from functools import lru_cache

import numpy as np
import pytest

from odom_ml import config as C
from odom_ml.data.bag import list_bags
from odom_ml.data.build import HZ, manifest
from odom_ml.data.splits import BagSplit, bag_splits
from odom_ml.geo import enu_to_utm
from odom_ml.position.pathgraph import load_pathgraph
from odom_ml.position.projection import MapProjection
from odom_ml.position.registration import build_registration

# the two receivers sit on the same rigid body; their separation is a property of
# the vehicle, so it is a reference the dataset itself provides
NOMINAL_BASELINE_M = 12.436
BASELINE_TOL_M = 0.35
UTM_ZONE = 37

RECEIVERS = {
    "master": (("gx", "gy", "gz"), "gnss_lat0", "gnss_lon0", "gnss_alt0"),
    "rover": (("rgx", "rgy", "rgz"), "gnss_rlat0", "gnss_rlon0", "gnss_ralt0"),
}


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line("markers", "dataset: needs the bag dataset on disk")
    config.addinivalue_line("markers", "slow: rebuilds or refits something expensive")


def require_dataset() -> None:
    if not C.BAGS_DIR.is_dir() or not list_bags():
        pytest.skip(f"bag dataset not available under {C.BAGS_DIR}; set ODOM_BAGS_DIR")


def require_cache() -> None:
    require_dataset()
    if not manifest(HZ)["count"]:
        pytest.skip("cache empty; run `python -m odom_ml.data.build` first")


@pytest.fixture(scope="session")
def dataset_bags() -> list[str]:
    require_dataset()
    return list_bags()


@pytest.fixture(scope="session")
def all_bags() -> list[dict]:
    require_cache()
    return manifest(HZ)["bags"]


@pytest.fixture(scope="session")
def split() -> BagSplit:
    require_cache()
    sp = bag_splits(seed=0)
    sp.check()
    return sp


@pytest.fixture(scope="session")
def pathgraph():
    return load_pathgraph()


def track_utm(bag_id: str, receiver: str = "master") -> tuple[np.ndarray, np.ndarray]:
    """Easting/northing of one receiver's track, in UTM.

    Reads only the six arrays this needs rather than the whole cache: a run holds
    ~34 arrays and each one touched costs a zlib inflate.
    """
    from odom_ml.data.splits import RECEIVER_KEYS, receiver_subset

    spec = RECEIVER_KEYS[receiver]
    pos = tuple(spec["pos"])  # type: ignore[arg-type]
    d = receiver_subset(bag_id, receiver, HZ)
    ok = np.ones(d[pos[0]].size, dtype=bool)
    for c in pos:
        ok &= np.isfinite(d[c])
    enu = np.column_stack([d[pos[0]][ok], d[pos[1]][ok], d[pos[2]][ok]])
    e, n, _ = enu_to_utm(
        enu, float(d[spec["lat"]]), float(d[spec["lon"]]), float(d[spec["alt"]]), zone=UTM_ZONE
    )
    return e, n


@lru_cache(maxsize=None)
def antenna_separation(bag_id: str) -> float:
    """Median master-rover distance for one run, cached across the session.

    Each run costs a zlib inflate of six arrays, and a fleet-wide check repeats
    that for every bag; the answer is a single float, so caching the result is
    cheap where caching the tracks would not be.
    """
    em, nm = track_utm(bag_id, "master")
    er, nr = track_utm(bag_id, "rover")
    n = min(em.size, er.size)
    return float(np.median(np.hypot(em[:n] - er[:n], nm[:n] - nr[:n])))


def stack_track_utm(
    bag_ids: list[str], receiver: str = "master", max_per_bag: int = 4000
) -> np.ndarray:
    """Several tracks stacked into one easting/northing cloud."""
    chunks = []
    for bid in bag_ids:
        e, n = track_utm(bid, receiver)
        pts = np.column_stack([e, n])
        if pts.shape[0] > max_per_bag:
            pts = pts[:: int(np.ceil(pts.shape[0] / max_per_bag))]
        chunks.append(pts)
    return np.vstack(chunks)


@pytest.fixture(scope="session")
def train_split(split: BagSplit) -> list[str]:
    return split.train


@pytest.fixture(scope="session")
def test_split(split: BagSplit) -> list[str]:
    return split.test


@pytest.fixture(scope="session")
def projection() -> MapProjection:
    """Projection built from the shipped route and the saved registration."""
    from odom_ml.position.registration import registration_path

    if not registration_path().exists():
        pytest.skip("route_registration.json not built")
    return MapProjection.load()


@pytest.fixture(scope="session")
def projection_train_only(train_split, pathgraph) -> MapProjection:
    """A projection whose registration was fitted on the training bags only.

    Scoring the held-out bags against this is what makes the evaluation honest;
    a transform fitted on the bags it is then measured on would report the
    registration error of the fit itself.
    """
    target = stack_track_utm(train_split)
    reg = build_registration(pathgraph, target, trim=0.8)
    return MapProjection(pathgraph, reg)


def bag_topics(bag_id: str) -> set[str]:
    """Topic names from a bag's rosbag2 metadata, read without a YAML parser."""
    text = (C.BAGS_DIR / bag_id / "metadata.yaml").read_text(encoding="utf-8")
    return set(re.findall(r"name:\s*(/\S+)", text))


def receiver_usable(d: dict, receiver: str, min_points: int = 1000) -> bool:
    """Re-exported from the package so tests and training share one definition."""
    from odom_ml.data.splits import receiver_usable as _impl

    return _impl(d, receiver, min_points)


def reference_bags(
    receiver: str = "master", min_duration: float = 0.0, hz: float = HZ
) -> list[str]:
    from odom_ml.data.splits import reference_bags as _impl

    return _impl(receiver, min_duration, hz)


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    d = np.diff(np.concatenate([[0], mask.view(np.int8), [0]]))
    return list(zip(np.where(d == 1)[0].tolist(), np.where(d == -1)[0].tolist()))
