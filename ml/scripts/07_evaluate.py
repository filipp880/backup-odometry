from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from odom_ml import config as C
from odom_ml.data.build import HZ, load_labeled, manifest
from odom_ml.estimator import EstimatorConfig, OdomEstimator
from odom_ml.metrics import along_track_metrics, vel_metrics
from odom_ml.models.traction import TractionModel
from odom_ml.position import load_pathgraph

INIT_WINDOW = 2.0


def arc_length(xy: np.ndarray) -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1), axis=0)])


def gnss_init(d: dict, window: float = INIT_WINDOW):
    t = d["t"]
    m = np.isfinite(d["gx"]) & (t <= window)
    if m.sum() < 5:
        return None
    v0 = float(np.nanmedian(d["speed"][m]))
    gx = float(np.nanmedian(d["gx"][m]))
    gy = float(np.nanmedian(d["gy"][m]))
    gz = float(np.nanmedian(d["gz"][m]))
    theta = None
    mm = m & (d["speed"] > 0.7)
    if mm.sum() >= 5:
        theta = float(np.arctan2(np.nanmean(d["gy"][mm]), np.nanmean(d["gx"][mm])))
    return {"v0": v0, "theta0": theta, "x0": gx, "y0": gy, "z0": gz}


def replay(
    d: dict,
    model: TractionModel,
    cfg: EstimatorConfig,
    init_mode: str = "gnss",
    route=None,
) -> dict[str, np.ndarray]:
    est = OdomEstimator(model, cfg, route=route)
    init = gnss_init(d) if init_mode == "gnss" else None
    if init is not None and init["theta0"] is not None:
        est.init_from_gnss(init["v0"], init["theta0"], init["x0"], init["y0"], init["z0"])
    elif init is not None:
        est.init_from_gnss(init["v0"], 0.0, init["x0"], init["y0"], init["z0"])
    elif init_mode == "wheel":
        v0 = float(np.nanmedian(d["v_wheel_mean"][:20]))
        est.init_from_gnss(v0, 0.0, 0.0, 0.0, 0.0)

    t = d["t"]
    n = t.size
    v = np.zeros(n)
    x = np.zeros(n)
    y = np.zeros(n)
    z = np.zeros(n)
    s = np.zeros(n)
    slip = np.zeros(n)
    trust = np.ones(n)
    disagree = np.zeros(n)
    slip_flag = np.zeros(n)
    u = d["u"]
    vf = d["v_front"]
    vr = d["v_rear"]
    for i in range(n):
        st = est.step(float(t[i]), u[i], vf[i], vr[i])
        v[i] = st.v
        x[i] = st.x
        y[i] = st.y
        z[i] = st.z
        s[i] = st.s
        slip[i] = st.slip
        trust[i] = st.trust
        disagree[i] = float(st.flags.get("disagree", False))
        slip_flag[i] = float(st.flags.get("slip", False))
    return {
        "v": v,
        "x": x,
        "y": y,
        "z": z,
        "s": s,
        "slip": slip,
        "trust": trust,
        "disagree": disagree,
        "slip_flag": slip_flag,
    }


def evaluate_bags(bag_ids, model, cfg, init_mode="gnss", route=None, label=""):
    rows = []
    for bid in bag_ids:
        d = load_labeled(bid, HZ)
        if d["t"].size < 200:
            continue
        out = replay(d, model, cfg, init_mode, route)
        ref = d["speed"]
        moving = ref > 0.5
        pos_ref = np.column_stack([d["gx"], d["gy"], d["gz"]])
        pos_est = np.column_stack([out["x"], out["y"], out["z"]])
        s_ref = arc_length(pos_ref[:, :2])
        m = np.isfinite(ref) & moving
        row = {
            "bag": bid,
            "vehicle": str(d["vehicle"]),
            "dur": float(d["t"][-1]),
            "traveled": float(s_ref[-1]),
            "wheel_rmse": vel_metrics(d["v_wheel_mean"], ref, m)["rmse"],
            "est_rmse": vel_metrics(out["v"], ref, m)["rmse"],
            "est_mae": vel_metrics(out["v"], ref, m)["mae"],
            "est_bias": vel_metrics(out["v"], ref, m)["bias"],
            "s_err_rmse": float(np.sqrt(np.mean((out["s"][m] - s_ref[m]) ** 2))),
            "s_err_final": float(out["s"][m][-1] - s_ref[m][-1]),
            "drift_pct": float(100.0 * (out["s"][m][-1] - s_ref[m][-1]) / max(s_ref[m][-1], 1e-6)),
            "slip_frac": float(np.mean(out["slip_flag"])),
            "disagree_frac": float(np.mean(out["disagree"])),
        }
        row["pos"] = along_track_metrics(pos_est, pos_ref)
        rows.append(row)
        print(
            f"  {bid} dur={row['dur']:7.1f} s={row['traveled']:7.1f}m wheel={row['wheel_rmse']:.3f} "
            f"est={row['est_rmse']:.3f} bias={row['est_bias']:+.3f} drift={row['drift_pct']:+.2f}% "
            f"pos3d={row['pos'].get('rmse_3d', float('nan')):8.1f}m slip={row['slip_frac']*100:4.1f}%"
        )
    return rows


def summarize(rows) -> dict:
    def agg(key, fn=np.nanmedian):
        vals = [r[key] for r in rows if key in r]
        return float(fn(vals)) if vals else float("nan")

    out = {
        "n_bags": len(rows),
        "total_dur": float(np.sum([r["dur"] for r in rows])),
        "total_traveled": float(np.sum([r["traveled"] for r in rows])),
        "wheel_rmse_med": agg("wheel_rmse"),
        "est_rmse_med": agg("est_rmse"),
        "est_rmse_mean": agg("est_rmse", np.nanmean),
        "est_mae_med": agg("est_mae"),
        "est_bias_med": agg("est_bias"),
        "s_err_rmse_med": agg("s_err_rmse"),
        "drift_pct_med": agg("drift_pct"),
        "drift_pct_mean": agg("drift_pct", np.nanmean),
        "pos_rmse3d_med": agg("pos", lambda x: np.nanmedian([p.get("rmse_3d", np.nan) for p in x])),
    }
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(C.ARTIFACTS_DIR / "models" / "traction_v1" / "traction.npz"))
    ap.add_argument("--bags", nargs="*", default=None)
    ap.add_argument("--limit", type=int, default=25)
    ap.add_argument("--init", default="gnss", choices=["gnss", "wheel", "zero"])
    ap.add_argument("--map", action="store_true")
    ap.add_argument("--out", default=str(C.WORK_DIR / "eval.json"))
    args = ap.parse_args()

    model = TractionModel.from_npz(Path(args.model))
    cfg = EstimatorConfig()
    route = None
    if args.map:
        route = load_pathgraph()
        cfg.use_map = True

    bag_ids = args.bags
    if not bag_ids:
        man = manifest(HZ)
        bag_ids = [r["bag_id"] for r in man["bags"] if r["duration"] >= 120.0][: args.limit]
    print(f"evaluating {len(bag_ids)} bags, init={args.init}, map={args.map}")
    rows = evaluate_bags(bag_ids, model, cfg, args.init, route)
    summary = summarize(rows)
    print("\nSUMMARY")
    for k, v in summary.items():
        print(f"  {k:22s} {v}")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nsaved -> {out}")


if __name__ == "__main__":
    main()
