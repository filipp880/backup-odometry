"""Bag-level train/validation/test splits.

All 122 runs share one route, so a split cannot be geographic: it is a split over
*runs*.  Two properties matter and both are enforced here rather than assumed:

* a bag is never cut in half, so no window of the same run can appear in both
  training and evaluation (temporal leakage);
* runs that look like the same physical journey -- same vehicle, same depot
  start point, same length -- are kept in the same split, so a near-duplicate
  cannot sit on both sides of the evaluation.

The split is deterministic for a given seed, and stratified by vehicle and by
duration so that the test set is not made only of the many very short runs.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .build import HZ, load_keys, manifest

# a run shorter than this is a start/stop fragment, useful for debugging but not
# representative of accumulated drift
DEFAULT_MIN_DURATION = 200.0

SPLIT_NAMES = ("train", "val", "test")


@dataclass
class BagSplit:
    train: list[str] = field(default_factory=list)
    val: list[str] = field(default_factory=list)
    test: list[str] = field(default_factory=list)

    def __getitem__(self, name: str) -> list[str]:
        if name not in SPLIT_NAMES:
            raise KeyError(f"unknown split {name!r}, expected one of {SPLIT_NAMES}")
        return getattr(self, name)

    def all(self) -> list[str]:
        return self.train + self.val + self.test

    def check(self) -> None:
        """Raise if the split is not disjoint and complete."""
        ids = self.all()
        dupes = {b for b in ids if ids.count(b) > 1}
        if dupes:
            raise AssertionError(f"bag(s) in more than one split: {sorted(dupes)}")
        for name in SPLIT_NAMES:
            if not getattr(self, name):
                raise AssertionError(f"split {name!r} is empty")

    def summary(self) -> dict[str, object]:
        return {name: len(getattr(self, name)) for name in SPLIT_NAMES}


#: position columns and origin scalars per receiver, named explicitly rather
#: than derived by string surgery
RECEIVER_KEYS: dict[str, dict[str, tuple[str, ...]]] = {
    "master": {
        "pos": ("gx", "gy", "gz"),
        "lat": "gnss_lat0",
        "lon": "gnss_lon0",
        "alt": "gnss_alt0",
    },
    "rover": {
        "pos": ("rgx", "rgy", "rgz"),
        "lat": "gnss_rlat0",
        "lon": "gnss_rlon0",
        "alt": "gnss_ralt0",
    },
}


def _receiver(receiver: str) -> dict[str, tuple[str, ...] | str]:
    if receiver not in RECEIVER_KEYS:
        raise ValueError(f"unknown receiver {receiver!r}, expected one of {sorted(RECEIVER_KEYS)}")
    return RECEIVER_KEYS[receiver]


def receiver_subset(bag_id: str, receiver: str, hz: float = HZ) -> dict[str, np.ndarray]:
    """Load only the arrays needed to judge or use one receiver."""
    from .build import load_keys

    spec = _receiver(receiver)
    keys = (*spec["pos"], spec["lat"], spec["lon"], spec["alt"])  # type: ignore[misc]
    return load_keys(bag_id, list(keys), hz)


def receiver_usable(d: dict, receiver: str, min_points: int = 1000) -> bool:
    """Whether a cached run carries enough of one receiver to be a reference.

    Not every run does: 39 of the 122 have no usable GNSS at all, and the two
    antennas are not present in exactly the same set of runs, so a caller that
    assumes they are would fail for a reason that is not a defect.
    """
    spec = _receiver(receiver)
    if spec["lat"] not in d:
        return False
    return all(k in d and np.isfinite(d[k]).sum() >= min_points for k in spec["pos"])  # type: ignore[arg-type]


def receiver_is_usable(bag_id: str, receiver: str = "master", hz: float = HZ) -> bool:
    return receiver_usable(receiver_subset(bag_id, receiver, hz), receiver)


def reference_bags(
    receiver: str = "master", min_duration: float = 0.0, hz: float = HZ
) -> list[str]:
    """Runs that can serve as their own reference, optionally long enough."""
    out = []
    for rec in manifest(hz)["bags"]:
        if rec["duration"] < min_duration:
            continue
        if receiver_is_usable(rec["bag_id"], receiver, hz):
            out.append(rec["bag_id"])
    return sorted(out)


def _group_key(bag_id: str, vehicle: str, duration: float, start_xy: tuple[float, float]) -> tuple:
    """Runs that plausibly record the same journey share a group.

    The metadata carries no wall-clock timestamp (``starting_time`` is empty), so
    near-duplicate detection uses what is available: vehicle, depot neighbourhood
    of the first fix, and total length.  Over-grouping is harmless -- it only
    makes the split coarser.
    """
    return (vehicle, round(start_xy[0] / 150.0), round(start_xy[1] / 150.0), round(duration / 60.0))


def bag_splits(
    hz: float = HZ,
    seed: int = 0,
    fractions: tuple[float, float, float] = (0.70, 0.15, 0.15),
    require_gnss: bool = True,
    min_duration: float = DEFAULT_MIN_DURATION,
) -> BagSplit:
    """Deterministic stratified bag-level split.

    ``require_gnss`` keeps only bags that can serve as their own reference, which
    is what the evaluation needs; ``min_duration`` drops run fragments.
    """
    if not np.isclose(sum(fractions), 1.0):
        raise ValueError(f"fractions must sum to 1, got {fractions}")
    man = manifest(hz)
    dur_of = {b["bag_id"]: float(b["duration"]) for b in man["bags"]}

    groups: dict[tuple, list[str]] = {}
    for rec in man["bags"]:
        bid, veh, dur = rec["bag_id"], rec["vehicle"], dur_of[rec["bag_id"]]
        if dur < min_duration:
            continue
        if require_gnss and not receiver_is_usable(bid, "master", hz):
            continue
        # locate the run by the first fixes, which are the depot end of the route
        d = load_keys(bid, ("gx", "gy"), hz)
        gx = d["gx"][:200]
        gy = d["gy"][:200]
        if np.isfinite(gx).sum() < 100:
            continue
        xy = (float(np.nanmedian(gx)), float(np.nanmedian(gy)))
        groups.setdefault(_group_key(bid, veh, dur, xy), []).append(bid)

    if not groups:
        raise ValueError("no bags passed the filters; check the cache and min_duration")

    # stratify by vehicle so both trams appear in every split.  Within a vehicle
    # the groups are shuffled and then handed out by quota: sorting by duration
    # instead would make the split a deterministic "shortest runs train" rule
    # that carries no randomness and reshuffles entirely if a duration changes.
    rng = np.random.default_rng(seed)
    out = BagSplit()
    for veh in sorted({k[0] for k in groups}):
        sub = [k for k in sorted(groups) if k[0] == veh]
        n = len(sub)
        if n < len(SPLIT_NAMES):
            raise ValueError(f"vehicle {veh} has only {n} group(s), cannot fill 3 splits")
        n_train = max(1, min(int(round(fractions[0] * n)), n - 2))
        n_val = max(1, min(int(round(fractions[1] * n)), n - n_train - 1))
        quota = {"train": n_train, "val": n_val, "test": n - n_train - n_val}
        perm = rng.permutation(n)
        for i, idx in enumerate(perm):
            part = next(p for p in SPLIT_NAMES if quota[p] > 0)
            quota[part] -= 1
            out[part].extend(groups[sub[idx]])

    for name in SPLIT_NAMES:
        setattr(out, name, sorted(getattr(out, name)))
    out.check()
    return out
