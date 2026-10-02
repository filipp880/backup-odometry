from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from odom_ml import config as C
from odom_ml.data.build import HZ, load_labeled, manifest
from odom_ml.models.traction import (
    DV,
    NU,
    NV,
    V_MAX,
    U_MIN,
    TractionModel,
    enforce_traction_monotonicity,
    smooth_table,
)

A_LIMIT = 1.8
V_MIN = 0.3


def collect(min_duration: float = 60.0) -> dict[str, np.ndarray]:
    man = manifest(HZ)
    cols: dict[str, list[np.ndarray]] = {k: [] for k in ("u", "speed", "accel", "vw", "vehicle", "bag")}
    for row in man["bags"]:
        if row["duration"] < min_duration:
            continue
        d = load_labeled(row["bag_id"], HZ)
        cols["u"].append(d["u"].astype(np.float32))
        cols["speed"].append(d["speed"].astype(np.float32))
        cols["accel"].append(d["accel"].astype(np.float32))
        cols["vw"].append(d["v_wheel_mean"].astype(np.float32))
        cols["vehicle"].append(np.full(d["t"].size, row["vehicle"]))
        cols["bag"].append(np.full(d["t"].size, row["bag_id"]))
    return {k: np.concatenate(v) for k, v in cols.items()}


def valid_mask(d: dict[str, np.ndarray]) -> np.ndarray:
    return (
        np.isfinite(d["u"])
        & np.isfinite(d["speed"])
        & np.isfinite(d["accel"])
        & (np.abs(d["accel"]) < A_LIMIT)
        & (d["speed"] > V_MIN)
    )


def split_runs(bags: np.ndarray, vehicles: np.ndarray, holdout_frac: float = 0.2, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    test_runs: set[str] = set()
    for veh in np.unique(vehicles):
        runs = np.unique(bags[vehicles == veh])
        runs = runs[rng.permutation(runs.size)]
        n = max(1, int(round(len(runs) * holdout_frac)))
        test_runs.update(runs[:n].tolist())
    return np.array([b in test_runs for b in bags])


def binned_table(
    u: np.ndarray, v: np.ndarray, a: np.ndarray, weights: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    ui = np.clip(np.round(u).astype(int) - U_MIN, 0, NU - 1)
    vi = np.clip((v / DV).astype(int), 0, NV - 1)
    flat = ui * NV + vi
    n = NU * NV
    if weights is None:
        weights = np.ones_like(a)
    num = np.bincount(flat, weights=a * weights, minlength=n)
    den = np.bincount(flat, weights=weights, minlength=n)
    table = np.where(den > 0, num / np.maximum(den, 1e-9), np.nan)
    return table.reshape(NU, NV), den.reshape(NU, NV)


def fill_table(table: np.ndarray, counts: np.ndarray) -> np.ndarray:
    out = table.copy()
    known = np.isfinite(out) & (counts > 0)
    if not known.any():
        return np.zeros_like(out)
    for i in range(out.shape[0]):
        for j in range(out.shape[1]):
            if known[i, j]:
                continue
            ii = np.clip(i + np.arange(-NU, NU), 0, NU - 1)
            jj = np.clip(j + np.arange(-NV, NV), 0, NV - 1)
            vals = np.full(ii.size, np.nan)
            for a in range(ii.size):
                for b in range(jj.size):
                    if known[ii[a], jj[b]]:
                        vals[a] = out[ii[a], jj[b]]
            if np.isfinite(vals).any():
                out[i, j] = np.nanmedian(vals)
    still = ~np.isfinite(out)
    out[still] = 0.0
    return out


def fit_iterative(
    u: np.ndarray, v: np.ndarray, a: np.ndarray, rounds: int = 3, sigma: float = 0.12
) -> tuple[np.ndarray, dict]:
    w = np.ones_like(a)
    info: dict = {}
    for r in range(rounds):
        tab, cnt = binned_table(u, v, a, weights=w)
        tab = fill_table(tab, cnt)
        tab = smooth_table(tab, wu=1, wv=1)
        tab = enforce_traction_monotonicity(tab)
        from odom_ml.models.traction import table_lookup

        pred = table_lookup(tab, u, v)
        res = a - pred
        scale = 1.4826 * np.median(np.abs(res - np.median(res))) + 1e-3
        w = 1.0 / (1.0 + (res / (3.0 * scale)) ** 2)
        info = {"round": r, "sigma": float(scale), "inlier_frac": float((w > 0.5).mean())}
    return tab, info


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(C.ARTIFACTS_DIR / "models" / "traction_v1"))
    ap.add_argument("--holdout", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    d = collect()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ok = valid_mask(d)
    print(f"samples={ok.size} valid={ok.mean():.3f} ({int(ok.sum())})")
    is_test = split_runs(d["bag"], d["vehicle"], args.holdout, args.seed)
    tr = ok & ~is_test
    te = ok & is_test
    print(f"train={int(tr.sum())} test={int(te.sum())} runs_test={len(np.unique(d['bag'][is_test]))}")

    from odom_ml.models.traction import table_lookup

    results = {}
    for name, sel in (("global", tr),):
        tab, info = fit_iterative(d["u"][sel], d["speed"][sel], d["accel"][sel])
        res = d["accel"][te] - table_lookup(tab, d["u"][te], d["speed"][te])
        res_tr = d["accel"][sel] - table_lookup(tab, d["u"][sel], d["speed"][sel])
        results[name] = {"test_rmse": float(np.sqrt(np.mean(res**2))), "train_rmse": float(np.sqrt(np.mean(res_tr**2))), **info}
        print(f"[{name}] train_rmse={res_tr.std():.4f} test_rmse={np.sqrt(np.mean(res**2)):.4f} {info}")
        if name == "global":
            np.save(out / "tmp_global.npy", tab)

    tab = np.load(out / "tmp_global.npy")
    (out / "tmp_global.npy").unlink()

    for veh in ("30618", "30639"):
        sel = tr & (d["vehicle"] == veh)
        if sel.sum() < 10000:
            continue
        tab_v, info = fit_iterative(d["u"][sel], d["speed"][sel], d["accel"][sel])
        res = d["accel"][te] - table_lookup(tab_v, d["u"][te], d["speed"][te])
        print(f"[{veh}] n={int(sel.sum())} test_rmse={np.sqrt(np.mean(res**2)):.4f}")
        np.savez_compressed(out / f"traction_{veh}.npz", table=tab_v.astype(np.float32))

    model = TractionModel(table=tab)
    model.meta = {
        "hz": HZ,
        "a_limit": A_LIMIT,
        "v_min": V_MIN,
        "dv": DV,
        "v_max": V_MAX,
        "u_min": U_MIN,
        "train_samples": int(tr.sum()),
        "holdout_frac": args.holdout,
        "seed": args.seed,
        "fit": results,
    }
    model.to_npz(out / "traction.npz")
    (out / "traction_meta.json").write_text(json.dumps(model.meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nsaved -> {out}")
    for u in (1, 5, 10, 15, -1, -5, -10, -15):
        row = tab[u - U_MIN]
        vals = [f"{row[int(s / DV)]:+.2f}" for s in (0.5, 3, 6, 9, 12)]
        print(f"  u={u:+3d} a(v=0.5,3,6,9,12) = {' '.join(vals)}")


if __name__ == "__main__":
    main()
