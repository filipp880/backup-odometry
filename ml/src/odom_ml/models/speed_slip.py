"""Learned models for speed correction and slip detection.

Two heads share one causal feature matrix (see :mod:`odom_ml.data.features`):

``speed``
    a direct regression onto the residual ``v_gnss - v_wheel``.  Adding a
    positive residual to the wheel reading corrects it.  The raw wheel signal is
    already good to a few cm/s, so the model mostly removes the systematic
    traction/braking transient bias rather than learning the speed itself.

``slip``
    a classifier for "the wheel reading is currently untrustworthy", trained
    against ``|v_wheel - v_gnss| > noise``.  The published probability is what
    the runtime uses to down-weight the wheel channel and fall back on the
    model, which is what criterion 3 of the case asks for.

Both are deliberately small gradient-boosted stump ensembles.  The case is
scored on real-time behaviour and robustness, and a depth-3 ensemble over twenty
causal features is fast enough, inspectable, and — unlike a HistGradientBoosting
model — converts to ONNX through skl2onnx without a custom converter, which is
what lets the C++ node run the very same graph.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SLIP_NOISE_M_S = 0.01
SEED = 0


@dataclass
class SpeedSlipModel:
    """Speed residual regression plus a slip gate, with its feature contract."""

    speed_model: object
    slip_model: object
    feature_names: list[str]
    speed_rmse_m_s: float = float("nan")
    speed_mae_m_s: float = float("nan")
    speed_bias_m_s: float = float("nan")
    baseline_rmse_m_s: float = float("nan")
    slip_auc: float = float("nan")
    slip_recall: float = float("nan")
    slip_precision: float = float("nan")
    slip_threshold: float = 0.5
    speed_shrink: float = 0.5
    hz: float = 50.0

    def predict_speed(self, X: np.ndarray) -> np.ndarray:
        """Corrected speed: the wheel reading plus a shrunk learned residual."""
        wheel = X[:, self.feature_names.index("v_wheel")].astype(np.float64)
        return wheel + self.speed_shrink * self.speed_model.predict(X)

    def predict_slip_probability(self, X: np.ndarray) -> np.ndarray:
        return self.slip_model.predict_proba(X)[:, 1]

    def correct_speed(self, X: np.ndarray, slip_gate: bool = True) -> np.ndarray:
        """Blend the wheel channel with the model using the slip gate.

        Where the wheels are trusted the corrected reading is used; where the gate
        fires the model prediction alone carries the estimate, so a spinning or
        locked wheel cannot drag the speed with it.
        """
        corrected = self.predict_speed(X)
        if not slip_gate:
            return corrected
        p = self.predict_slip_probability(X)
        trust = (p < self.slip_threshold).astype(np.float64)
        return trust * corrected + (1.0 - trust) * np.maximum(corrected, 0.0)


def fit_speed_slip(
    table,
    train_mask: np.ndarray,
    val_mask: np.ndarray | None = None,
    seed: int = SEED,
    speed_shrink: float = 0.5,
) -> SpeedSlipModel:
    """Fit both heads, defaulting to a conservative speed correction.

    The wheel speed is already within ~0.1 m/s of the reference, so an
    unconstrained regressor trained on millions of samples mostly learns the
    noise in the GNSS label and *degrades* the estimate.  Two guards keep the
    correction honest: it is fitted only where a transient could plausibly cause
    slip, and the fitted residual is shrunk toward zero by ``speed_shrink``.  The
    slip classifier is unaffected -- that is where the model earns its keep.
    """
    from sklearn.ensemble import HistGradientBoostingClassifier, HistGradientBoostingRegressor
    from sklearn.metrics import precision_score, recall_score, roc_auc_score

    names = table.names
    n = len(table)

    # A mask must be aligned with the table it indexes.  Fitting on one table and
    # validating with a mask built from another is silent corruption until numpy
    # raises deep inside, so refuse it here with a message that names the cause.
    for label, m in (("train_mask", train_mask), ("val_mask", val_mask)):
        if m is None:
            continue
        m = np.asarray(m)
        if m.dtype != np.bool_:
            raise TypeError(f"{label} must be a boolean mask, got {m.dtype}")
        if m.shape != (n,):
            raise ValueError(
                f"{label} has {m.shape[0]} entries but the table has {n} samples; "
                f"a mask may only index the table it was built from "
                f"(validate on a table built from the validation runs, not on the fit table)"
            )
        if m.sum() == 0:
            raise ValueError(f"{label} selects no samples")

    y_speed = table.slip  # wheel - truth, so truth = wheel - slip
    y_slip = (np.abs(table.slip) > SLIP_NOISE_M_S).astype(int)

    # index the table once: every table.X[mask] is a full copy of a 2M x 20
    # float32 array, and repeating it was most of the resident memory
    X_all = table.X
    a_all = X_all[:, names.index("a_fast")].astype(np.float64)
    u_all = X_all[:, names.index("u")].astype(np.float64)
    transient = (np.abs(a_all) > 0.3) | (np.abs(u_all) > 3.0)
    fit_idx = np.flatnonzero(train_mask & transient)
    if fit_idx.size < 100:
        raise ValueError(f"only {fit_idx.size} transient training samples; cannot fit")
    X_train = X_all[train_mask]
    X_fit = X_all[fit_idx]

    # HistGradientBoosting, not GradientBoosting: it is histogram-binned and
    # OpenMP-parallel, so 2M samples train in seconds instead of tens of minutes.
    # early_stopping must be off -- its default 'auto' carves an internal
    # validation split out of the fit data, which would leak the runs we report
    # on.  There is no subsample knob in HGB; depth is capped at 2 instead.
    common = dict(
        max_iter=200,
        learning_rate=0.15,
        max_depth=2,
        min_samples_leaf=100,
        l2_regularization=1.0,
        early_stopping=False,
        random_state=seed,
    )
    speed_model = HistGradientBoostingRegressor(**common)
    speed_model.fit(X_fit, y_speed[fit_idx])

    slip_model = HistGradientBoostingClassifier(**common)
    slip_model.fit(X_train, y_slip[train_mask])

    model = SpeedSlipModel(
        speed_model=speed_model,
        slip_model=slip_model,
        feature_names=names,
        speed_shrink=speed_shrink,
        hz=table.spec.hz,
    )

    if val_mask is not None and val_mask.sum() > 0:
        wheel = table.X[val_mask][:, names.index("v_wheel")].astype(np.float64)
        truth = table.v_true[val_mask]
        err = model.predict_speed(table.X[val_mask]) - truth
        model.speed_rmse_m_s = float(np.sqrt((err**2).mean()))
        model.speed_mae_m_s = float(np.abs(err).mean())
        model.speed_bias_m_s = float(err.mean())
        model.baseline_rmse_m_s = float(np.sqrt(((wheel - truth) ** 2).mean()))
        p = model.predict_slip_probability(table.X[val_mask])
        y = y_slip[val_mask]
        if len(np.unique(y)) > 1:
            model.slip_auc = float(roc_auc_score(y, p))
            pred_y = (p >= model.slip_threshold).astype(int)
            model.slip_recall = float(recall_score(y, pred_y, zero_division=0))
            model.slip_precision = float(precision_score(y, pred_y, zero_division=0))
    return model


def save_model(path: Path, model: SpeedSlipModel) -> Path:
    import joblib

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(model, path)
    meta = {
        "feature_names": model.feature_names,
        "speed_rmse_m_s": model.speed_rmse_m_s,
        "speed_mae_m_s": model.speed_mae_m_s,
        "speed_bias_m_s": model.speed_bias_m_s,
        "baseline_rmse_m_s": model.baseline_rmse_m_s,
        "slip_auc": model.slip_auc,
        "slip_recall": model.slip_recall,
        "slip_precision": model.slip_precision,
        "slip_threshold": model.slip_threshold,
        "hz": model.hz,
    }
    (path.parent / (path.stem + "_meta.json")).write_text(
        json.dumps(meta, indent=2), encoding="utf-8"
    )
    return path


def load_model(path: Path) -> SpeedSlipModel:
    import joblib

    return joblib.load(Path(path))
