"""Train the speed-correction and slip models, then report on held-out runs.

    python ml/scripts/20_train_speed_slip.py [--out artifacts/models/speed_slip]

The split is by run, never by sample, so the reported numbers come from driving
patterns the models were not fitted on.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from odom_ml.data.splits import bag_splits, reference_bags
from odom_ml.data.training import build_table, save_table, training_mask
from odom_ml.models.speed_slip import fit_speed_slip, save_model

import sys
from pathlib import Path as _P

sys.path.insert(0, str(_P(__file__).resolve().parents[1] / "src"))


def _peak_rss_mb() -> float:
    """Peak resident memory, best effort.

    ``resource`` only exists on Unix, so Windows goes through psapi; when neither
    is available the stage is still logged, just without the memory figure.
    """
    try:
        import resource

        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    except ImportError:
        pass
    try:
        import ctypes
        from ctypes import wintypes

        class _Counters(ctypes.Structure):
            _fields_ = [
                ("cb", wintypes.DWORD),
                ("PageFaultCount", wintypes.DWORD),
                ("PeakWorkingSetSize", ctypes.c_size_t),
                ("WorkingSetSize", ctypes.c_size_t),
                ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                ("PagefileUsage", ctypes.c_size_t),
                ("PeakPagefileUsage", ctypes.c_size_t),
            ]

        c = _Counters()
        c.cb = ctypes.sizeof(c)
        if ctypes.windll.psapi.GetProcessMemoryInfo(
            ctypes.windll.kernel32.GetCurrentProcess(), ctypes.byref(c), c.cb
        ):
            return c.PeakWorkingSetSize / (1024.0 * 1024.0)
    except Exception:  # noqa: BLE001
        pass
    return float("nan")


def log(msg: str) -> None:
    """Progress line, flushed immediately so a stall is visible in the log."""
    import time

    mb = _peak_rss_mb()
    tail = "" if mb != mb else f"  (peak rss {mb:.0f} MB)"
    print(f"[{time.strftime('%H:%M:%S')}] {msg}{tail}", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="artifacts/models/speed_slip")
    ap.add_argument("--tables", default="artifacts/tables")
    args = ap.parse_args()

    log("stage 1/6: bag split")
    sp = bag_splits(seed=args.seed)
    print("split:", sp.summary())
    print("train runs", len(sp.train), "val runs", len(sp.val), "test runs", len(sp.test))

    usable = set(reference_bags("master"))
    train_ids = [b for b in sp.train if b in usable]
    val_ids = [b for b in sp.val if b in usable]
    test_ids = [b for b in sp.test if b in usable]
    print(f"with a GNSS reference: train {len(train_ids)} val {len(val_ids)} test {len(test_ids)}")

    log(f"stage 2/6: building training table from {len(train_ids)} runs")
    table = build_table(train_ids)
    print(f"  train samples {len(table)}")

    log(f"stage 3/6: building validation table from {len(val_ids)} runs")
    val_table = build_table(val_ids) if val_ids else None
    train_mask = training_mask(table)
    val_mask = training_mask(val_table) if val_table is not None else None

    # fit_speed_slip indexes masks against the table it is given, so the
    # validation mask -- which belongs to val_table -- must not be passed in.
    # The validation metrics are computed below on val_table instead, which also
    # keeps every reported number derived from runs the fit never saw.
    log(f"stage 4/6: fitting both heads on {int(train_mask.sum())} samples")
    model = fit_speed_slip(table, train_mask, val_mask=None, seed=args.seed)
    log("          fit done")

    # choose the shrink factor on the validation runs, never on the fit data
    if val_table is not None and val_mask.sum() > 0:
        log("stage 5/6: shrink sweep on validation")
        wheel_v = val_table.X[val_mask][:, val_table.names.index("v_wheel")].astype(np.float64)
        truth_v = val_table.v_true[val_mask]
        base_v = float(np.sqrt(((wheel_v - truth_v) ** 2).mean()))
        best = (base_v, 0.0)
        print("\nshrink sweep on validation runs:")
        for s in (0.0, 0.25, 0.5, 0.75, 1.0):
            model.speed_shrink = s
            r = float(np.sqrt(((model.predict_speed(val_table.X[val_mask]) - truth_v) ** 2).mean()))
            mark = ""
            if r < best[0]:
                best = (r, s)
            print(f"  shrink {s:4.2f}  RMSE {r:.4f} m/s{mark}")
        print(f"  baseline (no correction) RMSE {base_v:.4f} m/s")
        model.speed_shrink = best[1]
        print(f"  -> chose shrink {best[1]} (val RMSE {best[0]:.4f})")
        if best[1] == 0.0:
            print("  -> the wheel reading is already optimal; the corrector is disabled")

    # final metrics on validation with the chosen shrink
    if val_table is not None and val_mask.sum() > 0:
        wheel = val_table.X[val_mask][:, val_table.names.index("v_wheel")].astype(np.float64)
        truth = val_table.v_true[val_mask]
        err = model.predict_speed(val_table.X[val_mask]) - truth
        model.baseline_rmse_m_s = float(np.sqrt(((wheel - truth) ** 2).mean()))
        model.speed_rmse_m_s = float(np.sqrt((err**2).mean()))
        model.speed_mae_m_s = float(np.abs(err).mean())
        model.speed_bias_m_s = float(err.mean())
        from sklearn.metrics import roc_auc_score

        p = model.predict_slip_probability(val_table.X[val_mask])
        y = (np.abs(val_table.slip[val_mask]) > 0.01).astype(int)
        model.slip_auc = float(roc_auc_score(y, p))
        from sklearn.metrics import precision_score, recall_score

        py = (p >= model.slip_threshold).astype(int)
        model.slip_recall = float(recall_score(y, py, zero_division=0))
        model.slip_precision = float(precision_score(y, py, zero_division=0))

    print("\nvalidation runs (unseen in fit):")
    print(f"  baseline wheel RMSE {model.baseline_rmse_m_s:.4f} m/s")
    print(f"  corrected   RMSE     {model.speed_rmse_m_s:.4f} m/s  MAE {model.speed_mae_m_s:.4f}  bias {model.speed_bias_m_s:+.4f}")
    print(f"  slip gate   AUC {model.slip_auc:.3f}  recall {model.slip_recall:.3f}  precision {model.slip_precision:.3f}")

    # honest evaluation on the test runs, unseen in fitting
    if test_ids:
        te = build_table(test_ids)
        mte = training_mask(te)
        wheel = te.X[mte][:, te.names.index("v_wheel")].astype(np.float64)
        truth = te.v_true[mte]
        pred = model.predict_speed(te.X[mte])
        err = pred - truth
        base = np.sqrt(((wheel - truth) ** 2).mean())
        rmse = np.sqrt((err**2).mean())
        p = model.predict_slip_probability(te.X[mte])
        y = (np.abs(te.slip[mte]) > 0.01).astype(int)
        from sklearn.metrics import roc_auc_score

        print("\nheld-out (test runs, unseen):")
        print(f"  samples {mte.sum()}  runs {len(test_ids)}")
        print(f"  baseline wheel RMSE {base:.4f} m/s")
        print(f"  corrected   RMSE     {rmse:.4f} m/s  MAE {np.abs(err).mean():.4f}  bias {err.mean():+.4f}")
        print(f"  slip gate   AUC {roc_auc_score(y, p):.3f}")

    out = Path(args.out)
    save_model(out / "speed_slip.joblib", model)
    save_table(Path(args.tables) / "speed_slip_table.npz", table,
               meta={"spec": table.spec.as_dict(), "n": len(table), "train_runs": len(train_ids)})
    print(f"\nsaved model -> {out/'speed_slip.joblib'}")


if __name__ == "__main__":
    main()
