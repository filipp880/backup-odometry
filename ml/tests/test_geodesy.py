"""Reference tests for the WGS84 -> UTM -> MGRS conversions.

The MGRS grid reference is only a naming of the UTM grid, but the jury frame is
documented as "MGRS", so the conversions must round-trip and the UTM maths must
be metrically correct.
"""

import numpy as np
import pytest

from odom_ml.geo import (
    dominant_utm_zone,
    ecef_to_enu,
    ecef_to_geodetic,
    enu_to_utm,
    geodetic_to_ecef,
    latlon_to_local_enu,
    latlon_to_mgrs,
    latlon_to_utm,
    local_enu_to_latlon,
    mgrs_to_utm,
    utm_zone,
)

# (lat, lon) pairs covering both hemispheres, several zones and the zone edges.
PTS = [
    (55.805, 37.421),  # tram area, zone 37N
    (55.75, 37.6173),
    (55.0, 37.0),
    (55.9, 37.0),
    (55.5, 38.5),  # near the 37/38 zone boundary
    (-33.9, 151.2),  # zone 56S
    (-1.0, 30.0),  # zone 36S
    (60.0, 179.9),  # zone 60N
    (0.0, 3.0),  # exactly on the equator on the zone 31 CM
]


def test_utm_zone_boundaries():
    assert utm_zone(np.array([-180.0])) == 1
    assert utm_zone(np.array([-174.1])) == 1
    assert utm_zone(np.array([-174.0])) == 2
    assert utm_zone(np.array([0.0])) == 31
    assert utm_zone(np.array([179.9])) == 60


def test_dominant_zone_uses_mode_not_mean():
    # 4 points in zone 37, 1 point in zone 31: the mean would round to 37 but
    # per-point zones must still be correct.
    lon = np.array([37.421, 37.5, 37.6, 37.7, 0.0])
    assert dominant_utm_zone(lon) == 37
    assert utm_zone(lon).tolist() == [37, 37, 37, 37, 31]
    assert latlon_to_utm(np.array([55.0] * 5), lon)[2].tolist() == [37, 37, 37, 37, 31]


def test_utm_is_metric_at_tram_latitude():
    e = latlon_to_utm(np.array([55.0, 55.0]), np.array([37.0, 37.01]))[0]
    dn = latlon_to_utm(np.array([55.0, 55.01]), np.array([37.0, 37.0]))[1]
    # 0.01 deg of latitude is ~1113 m everywhere; of longitude it scales as cos(lat).
    assert dn[1] - dn[0] == pytest.approx(1113.0, abs=3.0)
    assert e[1] - e[0] == pytest.approx(639.0, abs=3.0)


def test_false_northing_shifts_only_the_northern_hemisphere():
    lat = np.array([55.805, -33.9])
    lon = np.array([37.421, 151.2])
    with_fn = latlon_to_utm(lat, lon, false_northing=True)[1]
    without = latlon_to_utm(lat, lon, false_northing=False)[1]
    assert with_fn[0] - without[0] == pytest.approx(1e7, abs=1e-6)
    assert with_fn[1] - without[1] == pytest.approx(0.0, abs=1e-6)


def test_mgrs_text_format():
    r = latlon_to_mgrs(np.array([55.805]), np.array([37.421]))
    text = str(r["text"][0])
    # "37DB 00990 87980": grid zone + 100 km square, then the in-square metres.
    gzd, e_sq, n_sq = text.split()
    assert gzd[:2] == "37"
    square = gzd[2:]
    assert len(square) == 2 and square.isalpha() and square.isupper()
    assert "I" not in text and "O" not in text
    assert len(e_sq) == 5 and len(n_sq) == 5
    assert e_sq.isdigit() and n_sq.isdigit()
    # in-square coordinates must stay inside their 100 km square
    assert 0 <= int(e_sq) < 100000
    assert 0 <= int(n_sq) < 100000


@pytest.mark.parametrize("lat,lon", PTS)
def test_mgrs_round_trip(lat, lon):
    r = latlon_to_mgrs(np.array([lat]), np.array([lon]))
    text = str(r["text"][0])
    zone = int(text[0:2])
    col, row = text[2], text[3]
    e_in, n_in = (int(v) for v in text.split()[1:3])
    e, n = mgrs_to_utm(zone, col, row, e_in, n_in, northing_ref=float(r["northing_utm"][0]))
    # MGRS truncates, so the inverse may only recover the 1 m square.
    assert e == pytest.approx(float(r["easting_utm"][0]), abs=1.0)
    assert n == pytest.approx(float(r["northing_utm"][0]), abs=1.0)


def test_mgrs_precision_truncates_not_rounds():
    r = latlon_to_mgrs(np.array([55.805]), np.array([37.421]), digits=3)
    assert len(str(r["text"][0]).split()[1]) == 3
    assert len(str(r["text"][0]).split()[2]) == 3


def test_mgrs_to_utm_rejects_bad_zone():
    with pytest.raises(ValueError):
        mgrs_to_utm(0, "D", "B", 990, 87980)
    with pytest.raises(ValueError):
        mgrs_to_utm(61, "D", "B", 990, 87980)


# --- ECEF / local ENU ---------------------------------------------------------

LAT0, LON0, ALT0 = 55.80482941166667, 37.420496525, 168.0
OFFSETS = [(55.80, 37.42), (55.81, 37.43), (55.80, 37.30), (55.75, 37.00), (55.9, 37.9)]


def test_ecef_geodetic_round_trip():
    lat = np.array([p[0] for p in OFFSETS])
    lon = np.array([p[1] for p in OFFSETS])
    alt = np.linspace(100.0, 200.0, lat.size)
    back = ecef_to_geodetic(geodetic_to_ecef(lat, lon, alt))
    m_per_deg = 111320.0
    assert np.allclose(back[0], lat, atol=1e-9)
    assert np.allclose(back[1], lon, atol=1e-9)
    assert np.allclose(back[2], alt, atol=1e-6)
    assert m_per_deg > 0.0


def test_enu_round_trip():
    lat = np.array([p[0] for p in OFFSETS])
    lon = np.array([p[1] for p in OFFSETS])
    alt = np.linspace(160.0, 175.0, lat.size)
    enu = latlon_to_local_enu(lat, lon, alt, LAT0, LON0, ALT0)
    back = local_enu_to_latlon(enu, LAT0, LON0, ALT0)
    assert np.allclose(back[0], lat, atol=1e-9)
    assert np.allclose(back[1], lon, atol=1e-9)
    assert np.allclose(back[2], alt, atol=1e-6)


def test_enu_matches_ecef_to_enu():
    lat = np.array([p[0] for p in OFFSETS])
    lon = np.array([p[1] for p in OFFSETS])
    alt = np.linspace(160.0, 175.0, lat.size)
    xyz = geodetic_to_ecef(lat, lon, alt)
    assert np.allclose(latlon_to_local_enu(lat, lon, alt, LAT0, LON0, ALT0), ecef_to_enu(xyz, LAT0, LON0, ALT0))


def test_enu_to_utm_is_close_to_enu_but_rotated():
    # The cached labels are true-north ENU; the jury frame is UTM grid axes, so
    # the two differ by the meridian convergence (about 1.3 deg here), not just
    # by an offset.  A pure offset would therefore be wrong by ~120 m over 5 km.
    lat = np.array([55.80, 55.80, 55.75, 55.90])
    lon = np.array([37.42, 37.62, 37.20, 37.90])
    enu = latlon_to_local_enu(lat, lon, np.full(lat.size, 168.0), LAT0, LON0, ALT0)
    e, n, _ = enu_to_utm(enu, LAT0, LON0, ALT0, zone=37)

    # UTM without the false northing, minus the UTM of the local origin.
    e0, n0, _ = latlon_to_utm(np.array([LAT0]), np.array([LON0]), zone=37, false_northing=False)
    du = (e - e0[0], n - n0[0])
    direct = np.hypot(enu[:, 0], enu[:, 1])  # enu is already relative to the local origin
    via_utm = np.hypot(*du)

    assert np.allclose(direct, via_utm, atol=2.0)  # same distances, rotated axes
    # the rotation is real: the axes are not aligned
    cosang = (enu[:, 0] * du[0] + enu[:, 1] * du[1]) / (direct * via_utm)
    assert np.degrees(np.arccos(np.clip(cosang, -1, 1))).max() > 0.5
