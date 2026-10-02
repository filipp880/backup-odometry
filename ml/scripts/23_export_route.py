"""Export the registered route as a flat UTM polyline, plus a scoring target.

The C++ node must not have to reimplement geodesy.  The supplied pathgraph lives
in an unknown local frame, so the consumer would otherwise need UTM, MGRS, the
registration, and a KD-tree projection.  After this step it needs one array scan:
find the nearest segment, walk it, done.

The second artefact is the reference trajectory in the jury's own frame for one
run, so the port can measure itself without anything from this repository.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from odom_ml.data.build import HZ, load_labeled  # noqa: E402
from odom_ml.export import dump_line_json, to_array  # noqa: E402
from odom_ml.geo import enu_to_utm, latlon_to_mgrs  # noqa: E402
from odom_ml.position.projection import MapProjection  # noqa: E402
from odom_ml.position.registration import (  # noqa: E402
    load_registration,
    registration_path,
)

UTM_ZONE = 37


def route_document(mp: MapProjection) -> dict:
    """The route in UTM, with heading, grade and curvature at every vertex.

    ``grade`` is the literal dz/ds the interface asks for, which is dimensionless
    rise-over-run and not an angle; ``grade_rad`` carries the same thing as an
    angle for a runtime that wants radians.  ``curvature`` is dtheta/ds in 1/m
    and is exactly zero on a straight section, so a radius derived from it is
    infinite there rather than undefined.
    """
    e, n = mp.map_to_utm(mp.graph.xy)
    s = mp.graph.s
    z = mp.graph.z
    tan = mp.tangent_at(s)  # unit heading in UTM
    nrm = np.column_stack([-tan[:, 1], tan[:, 0]])  # left normal

    heading = np.unwrap(np.arctan2(tan[:, 1], tan[:, 0]))
    dz_ds = np.gradient(z, s)
    grade_rad = np.arctan(dz_ds)
    curv = np.gradient(heading, s)
    reg = mp.reg
    return {
        "format": "odom_ml.route_utm/2",
        "conventions": {
            "units": "metres; heading in degrees and radians; grade is rise-over-run; "
                     "curvature in 1/m",
            "frame": f"UTM zone {UTM_ZONE}N, WGS84",
            "false_northing": (
                "DISABLED. Northings are around 6 188 000, not 16 188 000. Adding the "
                "10 000 km false northing is a constant that cancels once the start point "
                "is subtracted, but a port that mixes the two conventions will be wrong by "
                "10 000 km."
            ),
            "axes": (
                "x = easting, y = northing. UTM grid axes are not north-aligned: the meridian "
                "convergence here is about 1.3 deg, which is roughly 100 m of x error over "
                "5 km if the axes are confused with a tangent-plane ENU frame."
            ),
            "heading": "direction of increasing arclength, measured from east (+x) "
                       "counter-clockwise, unwrapped continuously along the route",
            "grade": "dz/ds, dimensionless. grade_rad = atan(grade) if an angle is wanted. "
                     "Positive means climbing.",
            "curvature": "d(heading)/ds in 1/m; exactly 0 on a straight section, where a "
                         "turn radius would be infinite",
            "normal": "left normal, so a positive cross-track offset is left of travel",
            "origin": (
                "The output frame is this polyline's UTM minus the UTM of the first GNSS fix "
                "of the run. The route is NOT the origin."
            ),
        },
        "route": {
            "source_name": reg.source_name,
            "n_points": int(s.size),
            "length_m": float(s[-1]),
            "endpoint_gap_m": float(np.linalg.norm(mp.graph.xy[-1] - mp.graph.xy[0])),
            "elevation_min_m": float(z.min()),
            "elevation_max_m": float(z.max()),
            "grade_abs_max": float(np.abs(dz_ds).max()),
            "curvature_min_per_m": float(curv.min()),
            "curvature_max_per_m": float(curv.max()),
            "min_turn_radius_m": float(1.0 / np.abs(curv).max()),
            "self_overlap": (
                "None: the two legs are on separate tracks, so a nearest-point arclength is "
                "unambiguous and no search window is needed."
            ),
        },
        "registration": {
            "rotation": reg.rotation.tolist(),
            "translation": reg.translation.tolist(),
            "direction": reg.direction,
            "scale_estimate": reg.scale_estimate,
            "scale_note": (
                "Close to 1, so both frames are metric. A fitted scale far from 1 would mean "
                "a units error in one of the two frames."
            ),
            "rms_residual_m": reg.rms_residual_m,
            "median_residual_m": reg.median_residual_m,
            "fitted_on": "GNSS tracks from the cached runs; see 12_build_registration.py",
            "artifact": str(registration_path()),
        },
        "columns": [
            "x", "y", "z", "s",
            "heading_deg", "heading_rad", "grade", "grade_rad", "curvature",
            "t_x", "t_y", "n_x", "n_y",
        ],
        "points": [
            [
                float(e[i]), float(n[i]), float(z[i]), float(s[i]),
                float(np.degrees(heading[i])), float(heading[i]),
                float(dz_ds[i]), float(grade_rad[i]), float(curv[i]),
                float(tan[i, 0]), float(tan[i, 1]), float(nrm[i, 0]), float(nrm[i, 1]),
            ]
            for i in range(s.size)
        ],
    }


def ground_truth_document(bag_id: str) -> dict:
    """The reference trajectory in the jury frame, so a port can score itself."""
    d = load_labeled(bag_id, HZ)
    ok = np.isfinite(d["gx"]) & np.isfinite(d["gy"]) & np.isfinite(d["gz"])
    idx = np.flatnonzero(ok)
    lat0, lon0, alt0 = (
        float(d["gnss_lat0"]), float(d["gnss_lon0"]), float(d["gnss_alt0"])
    )
    mgrs = latlon_to_mgrs(np.array([lat0]), np.array([lon0]))
    mp = MapProjection.load()

    # The reference trajectory is a *geodetic* conversion: the cached GNSS is a
    # true-north ENU frame, so ENU -> UTM through the ellipsoid.  It must NOT go
    # through the registration, which maps the pathgraph's own unknown frame into
    # UTM and is a completely different transform.  Applying both puts the track
    # thousands of kilometres away.
    enu = np.column_stack([d["gx"][idx], d["gy"][idx], d["gz"][idx]])
    truth_e, truth_n, _ = enu_to_utm(enu, lat0, lon0, alt0, zone=UTM_ZONE)
    truth = np.column_stack([truth_e, truth_n])
    rel = truth - truth[0]

    s = mp.project_utm(truth[:, 0], truth[:, 1])
    on_route = np.abs(s.lateral) < 15.0

    return {
        "format": "odom_ml.ground_truth/1",
        "bag_id": bag_id,
        "hz": HZ,
        "n_rows": int(idx.size),
        "conventions": {
            "frame": f"UTM zone {UTM_ZONE}N minus the first valid GNSS fix of the run",
            "false_northing": "disabled, as in route_utm.json",
            "origin_utm": [float(truth[0, 0]), float(truth[0, 1])],
            "origin_mgrs": str(mgrs["text"][0]),
            "origin_alt_m": alt0,
            "x": "easting minus origin, m",
            "y": "northing minus origin, m",
            "z": "altitude minus origin altitude, m -- NOTE the jury said z is a height; "
                 "whether they subtract the start altitude or compare absolute is not "
                 "resolved, this file uses relative",
            "t": "seconds from the first sample of the run",
        },
        "note": (
            "Scoring target, not an input. The vehicle spends part of the run outside the "
            "mapped corridor (depot), where the route offers no constraint; on_route marks "
            "which rows that is."
        ),
        "columns": ["t", "x", "y", "z", "s", "cross_track", "on_route"],
        "rows": [
            [float(d["t"][idx[i]]), float(rel[i, 0]), float(rel[i, 1]),
             float(d["gz"][idx[i]]), float(s.s[i]), float(s.lateral[i]),
             int(bool(on_route[i]))]
            for i in range(idx.size)
        ],
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="artifacts/route")
    ap.add_argument("--truth-out", default="artifacts/reference")
    ap.add_argument("--bag", default="30618_efb92709")
    ap.add_argument("--max-truth-rows", type=int, default=0)
    args = ap.parse_args()

    mp = MapProjection.load()
    reg = load_registration()
    print(f"registration: {reg.direction}, RMS {reg.rms_residual_m:.2f} m, "
          f"scale {reg.scale_estimate:.6f}")

    doc = route_document(mp)
    p = dump_line_json(doc, Path(args.out) / "route_utm.json")
    e = np.array([r[0] for r in doc["points"]])
    n = np.array([r[1] for r in doc["points"]])
    print(f"\nwrote {p}  ({p.stat().st_size/1024:.0f} KB)")
    print(f"  {doc['route']['n_points']} points, {doc['route']['length_m']:.1f} m")
    print(f"  E {e.min():.1f}..{e.max():.1f}   N {n.min():.1f}..{n.max():.1f}  "
          f"(false northing disabled)")

    truth = ground_truth_document(args.bag)
    if args.max_truth_rows and truth["n_rows"] > args.max_truth_rows:
        step = int(np.ceil(truth["n_rows"] / args.max_truth_rows))
        truth["rows"] = truth["rows"][::step]
        truth["n_rows"] = len(truth["rows"])
        truth["downsampled_by"] = step
    q = dump_line_json(truth, Path(args.truth_out) / f"ground_truth_{args.bag}.json")
    x = to_array([r[1] for r in truth["rows"]])
    y = to_array([r[2] for r in truth["rows"]])
    s = to_array([r[4] for r in truth["rows"]])
    on = np.array([r[6] for r in truth["rows"]])
    print(f"\nwrote {q}  ({q.stat().st_size/1024:.0f} KB)")
    print(f"  {truth['n_rows']} rows from {truth['bag_id']}, origin {truth['conventions']['origin_mgrs']}")
    print(f"  x {x.min():.0f}..{x.max():.0f} m, y {y.min():.0f}..{y.max():.0f} m, "
          f"s {s.min():.0f}..{s.max():.0f} m")
    print(f"  on-route {on.mean()*100:.1f} % of the run (the rest is depot, off the map)")


if __name__ == "__main__":
    main()
