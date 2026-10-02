"""Проверка, соответствует ли предоставленный JSON-маршрут GNSS-трекам.

Правильный ICP: вращение применяется один раз, к начальной оси, затем перебор.
"""
import json

import numpy as np

from odom_ml import config as C
from odom_ml.data.build import HZ, load_labeled, manifest

from odom_ml.position.pathgraph import load_pathgraph

raw = json.loads((C.DEFAULT_DATA_ROOT / "щукинская - таллинская.json").read_text(encoding="utf-8"))
route = load_pathgraph()
print("route loaded: n=%d length=%.1f  x %.0f..%.0f y %.0f..%.0f" % (route.xy.shape[0], route.length, route.xy[:, 0].min(), route.xy[:, 0].max(), route.xy[:, 1].min(), route.xy[:, 1].max()))
jxy = route.xy
print("json spacing 1.0 m, 4710 points, total 4708.3 m (metric local frame, no lat/lon)")


def rot(theta, p):
    c, s = np.cos(theta), np.sin(theta)
    return np.column_stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]])


def nn_dist(a, b):
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    j = np.argmin(d, axis=1)
    return d[np.arange(a.shape[0]), j], j


def icp(src, dst, iters=60, trim=0.6):
    """Возвращает (theta, trans) с однократным применением вращения к src."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - mu_s, dst - mu_d
    theta, prev = 0.0, np.inf
    for _ in range(iters):
        w = rot(theta, a)
        d, j = nn_dist(w, b)
        if d.mean() > prev:
            break
        prev = d.mean()
        keep = d < np.quantile(d, trim)
        if keep.sum() < 10:
            break
        w_k, b_k = w[keep], b[j[keep]]
        num = float(np.mean(b_k[:, 1] * w_k[:, 0] - b_k[:, 0] * w_k[:, 1]))
        den = float(np.mean(b_k[:, 0] * w_k[:, 0] + b_k[:, 1] * w_k[:, 1]))
        th = float(np.arctan2(num, den))
        theta += th
    t = dst.mean(axis=0) - rot(theta, (src - mu_s)).mean(axis=0)
    return theta, t


man = manifest(HZ)
bags = [r["bag_id"] for r in man["bags"] if r["duration"] >= 300.0][:12]
print("\n%-18s %8s %8s %8s %8s %9s" % ("bag", "n", "trk_len", "med_d", "p90_d", "route_med"))
for bid in bags:
    d = load_labeled(bid, HZ)
    g = np.column_stack([d["gx"], d["gy"]])
    g = g[np.isfinite(g).all(axis=1)]
    if g.shape[0] < 50:
        continue
    step = np.linalg.norm(np.diff(g, axis=0), axis=1)
    g = np.vstack([g[:1], g[1:][step < 20.0]])
    trk_len = float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum())
    # равномерная подвыборка для скорости
    if g.shape[0] > 4000:
        g = g[:: max(1, g.shape[0] // 4000)]
    # грубое совмещение: среднее трека -> ближайшая точка маршрута, затем ICP
    mu = g.mean(axis=0)
    mu_r = np.array([route.xy[:, 0].mean(), route.xy[:, 1].mean()])
    g0 = g - mu + mu_r
    th, tr = icp(g0, route.xy, iters=40)
    g_al = rot(th, g - mu) + tr
    d0, _ = nn_dist(g_al, route.xy)
    print(
        "%-18s %8d %8.0f %8.1f %8.1f %9.1f"
        % (bid, g.shape[0], trk_len, np.median(d0), np.percentile(d0, 90), float(np.median(np.linalg.norm(route.xy - route.xy.mean(axis=0), axis=1))))
    )
    print("      theta=%.4f deg  trans=(%.1f, %.1f)" % (np.degrees(th), tr[0], tr[1]))
