"""Training tables for the speed and slip models.

The labels come from the GNSS reference, which is what the jury itself uses to
score accuracy, and the features come only from the three permitted topics.  The
split between the two is the whole point: the model is *trained* against GNSS but
*runs* without it, which is why the feature builder never sees a GNSS column.

Two targets are produced per sample.

``slip``
    ``v_wheel - v_gnss`` in m/s.  Positive while the wheels spin up under
    traction, negative while they lock or slide under braking.  The task
    statement lists exactly these regimes as the failure modes to detect.

``v_true``
    the GNSS ground speed, i.e. what a correct speed estimate should output.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .. import config as C
from .build import HZ, load_labeled
from .features import FeatureSpec, build_features, feature_names

# only samples where the reference is meaningful are used as targets
MIN_GNSS_SPEED = 0.3
MIN_SAMPLES_FOR_VALID_RATE = 200


@dataclass
class TrainingTable:
    X: np.ndarray  #: (n, d) float32 features
    slip: np.ndarray  #: (n,) m/s, wheel minus truth
    v_true: np.ndarray  #: (n,) m/s
    bag_id: np.ndarray  #: (n,) source run, for grouped splitting
    t: np.ndarray  #: (n,) s from the start of the run
    spec: FeatureSpec

    def __len__(self) -> int:
        return int(self.X.shape[0])

    @property
    def names(self) -> list[str]:
        return list(self.spec.names)

    def subset(self, bag_ids: list[str]) -> TrainingTable:
        keep = np.isin(self.bag_id, np.asarray(bag_ids, dtype=object))
        return TrainingTable(
            X=self.X[keep],
            slip=self.slip[keep],
            v_true=self.v_true[keep],
            bag_id=self.bag_id[keep],
            t=self.t[keep],
            spec=self.spec,
        )


def build_table(
    bag_ids: list[str],
    hz: float = HZ,
    wheel_scale: float = 1.0,
) -> TrainingTable:
    """Concatenate the labelled samples of the given runs."""
    xs, slips, vtrues, bids, ts = [], [], [], [], []
    for bid in bag_ids:
        d = load_labeled(bid, hz)
        if "gnss_lat0" not in d or "speed" not in d:
            continue
        ok = (
            np.isfinite(d["u"])
            & np.isfinite(d["v_front"])
            & np.isfinite(d["v_rear"])
            & np.isfinite(d["speed"])
        )
        if ok.sum() < MIN_SAMPLES_FOR_VALID_RATE:
            continue
        x, spec = build_features(
            d["u"][ok], d["v_front"][ok], d["v_rear"][ok], hz=hz, wheel_scale=wheel_scale
        )
        v_wheel = x[:, feature_names().index("v_wheel")].astype(np.float64)
        v_ref = d["speed"][ok]
        xs.append(x)
        slips.append(v_wheel - v_ref)
        vtrues.append(v_ref)
        bids.append(np.full(int(ok.sum()), bid, dtype=object))
        ts.append(d["t"][ok])
    if not xs:
        raise ValueError("no run produced labelled samples")
    return TrainingTable(
        X=np.vstack(xs),
        slip=np.concatenate(slips),
        v_true=np.concatenate(vtrues),
        bag_id=np.concatenate(bids),
        t=np.concatenate(ts),
        spec=spec,
    )


def training_mask(table: TrainingTable) -> np.ndarray:
    """Samples usable for fitting: reference valid and the vehicle actually moving."""
    return np.isfinite(table.slip) & (table.v_true >= MIN_GNSS_SPEED)


def save_table(path: Path, table: TrainingTable, meta: dict[str, object] | None = None) -> Path:
    import json

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        X=table.X,
        slip=table.slip.astype(np.float32),
        v_true=table.v_true.astype(np.float32),
        bag_id=table.bag_id.astype(str),
        t=table.t.astype(np.float32),
        names=np.asarray(table.names, dtype=str),
        meta=np.asarray(json.dumps(meta or {"spec": table.spec.as_dict()}, default=str)),
    )
    return path
