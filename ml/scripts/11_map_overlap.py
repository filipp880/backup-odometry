"""Совпадает ли JSON-маршрут с GNSS-треками? (общая система UTM, robust ICP)"""
import numpy as np

from odom_ml.data.build import HZ, load_labeled, manifest
from odom_ml.geo.geodesy import latlon_to_utm
from odom_ml.position.pathgraph import load_pathgraph

route = load_pathgraph()
man = manifest(HZ)
bags = [r["bag_id"] for r in man["bags"] if r["duration"] >= 200.0]
print("bags", len(bags))

tracks = []
for bid in bags:
    d = load_labeled(bid, HZ)
    g = np.column_stack([d["gx"], d["gy"]])
    ok = np.isfinite(g).all(axis=1)
    if ok.sum() < 100:
        continue
    g = g[ok]
    step = np.linalg.norm(np.diff(g, axis=0), axis=1)
    g = np.vstack([g[:1], g[1:][step < 20.0]])
    if g.shape[0] > 2000:
        g = g[:: max(1, g.shape[0] // 2000)]
    lat0 = float(d["gnss_lat0"])
    lon0 = float(d["gnss_lon0"])
    alt0 = float(d["gnss_alt0"])
    e0, n0, _z = latlon_to_utm(np.array([lat0]), np.array([lon0]))
    tracks.append(g + np.array([float(e0[0]), float(n0[0])]))
allg = np.vstack(tracks)
print("tracks n=%d bbox E %.0f..%.0f N %.0f..%.0f" % (allg.shape[0], allg[:, 0].min(), allg[:, 0].max(), allg[:, 1].min(), allg[:, 1].max()))
print("route  bbox E %.0f..%.0f N %.0f..%.0f (in unknown local frame)" % (route.xy[:, 0].min(), route.xy[:, 0].max(), route.xy[:, 1].min(), route.xy[:, 1].max()))


def rot(theta, p):
    c, s = np.cos(theta), np.sin(theta)
    return np.column_stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]])


def nn(a, b, chunk=500):
    out = np.empty(a.shape[0])
    for i in range(0, a.shape[0], chunk):
        blk = a[i : i + chunk]
        d = np.linalg.norm(blk[:, None, :] - b[None, :, :], axis=2)
        out[i : i + chunk] = d.min(axis=1)
    return out


def icp2(src, dst, iters=60, trims=(0.7, 0.5, 0.35, 0.25, 0.2)):
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - mu_s, dst - mu_d
    theta = 0.0
    for trim in trims:
        prev = np.inf
        for _ in range(iters):
            w = rot(theta, a)
            dmat = np.linalg.norm(w[:, None, :] - b[None, :, :], axis=2)
            d = dmat.min(axis=1)
            if d.mean() > prev:
                break
            prev = d.mean()
            thr = np.quantile(d, trim)
            keep = d < thr
            if keep.sum() < 20:
                break
            j = np.argmin(dmat[keep], axis=1)
            w_k, b_k = w[keep], b[j]
            num = float(np.mean(b_k[:, 1] * w_k[:, 0] - b_k[:, 0] * w_k[:, 1]))
            den = float(np.mean(b_k[:, 0] * w_k[:, 0] + b_k[:, 1] * w_k[:, 1]))
            theta += float(np.arctan2(num, den))
    t = b.mean(axis=0) - rot(theta, a).mean(axis=0)
    return theta, t


def pca_frame(p):
    mu = p.mean(axis=0)
    c = p - mu
    w, v = np.linalg.eigh(c.T @ c)
    order = np.argsort(-w)
    return mu, v[:, order[0]], v[:, order[1]]


mu_r, ex_r, ey_r = pca_frame(route.xy)
mu_t, ex_t, ey_t = pca_frame(allg)
print("\nroute PCA axes ang %.2f/%.2f deg, tracks %.2f/%.2f deg" % (np.degrees(np.arctan2(ex_r[1], ex_r[0])), np.degrees(np.arctan2(ey_r[1], ey_r[0])), np.degrees(np.arctan2(ex_t[1], ex_t[0])), np.degrees(np.arctan2(ey_t[1], ey_t[0]))))

best = None
for sx in (1.0, -1.0):
    for sy in (1.0, -1.0):
        R0 = np.column_stack([sx * ex_t, sy * ey_t]).T
        th0 = np.arctan2(R0[1, 0], R0[0, 0])
        init = rot(th0, route.xy - mu_r) + mu_t
        th, t = icp2(init, allg)
        aligned = rot(th, route.xy - mu_r) + mu_t + t
        d = nn(aligned, allg)
        med = float(np.median(d))
        print("  init(sx=%.0f,sy=%.0f) theta0=%7.2f -> theta=%7.2f  med=%7.1f p90=%7.1f" % (sx, sy, np.degrees(th0), np.degrees(th), med, np.percentile(d, 90)))
        if best is None or med < best[0]:
            best = (med, th, t, d, aligned)

med, th, t, d, aligned = best
print("\nBEST: median route->track distance = %.1f m  (p90 %.1f, frac<10m %.3f, frac<20m %.3f)" % (med, np.percentile(d, 90), (d < 10).mean(), (d < 20).mean()))
print("rotation applied to route: %.3f deg, translation (%.0f, %.0f)" % (np.degrees(th), t[0], t[1]))
print("track->route median: %.1f" % np.median(nn(allg, aligned)))
