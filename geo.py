"""WGS84 ECEF/ENU helpers for comparing a live fix against a fixed fiducial
(reference) point -- used by ntrip_caster.py's fiducial/correction TUI panel."""
import math

WGS84_A = 6378137.0            # semi-major axis, meters
WGS84_F = 1 / 298.257223563    # flattening
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def lla_to_ecef(lat_deg, lon_deg, height_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height_m) * cos_lat * cos_lon
    y = (n + height_m) * cos_lat * sin_lon
    z = (n * (1 - WGS84_E2) + height_m) * sin_lat
    return x, y, z


def ecef_delta_to_enu(dx, dy, dz, ref_lat_deg, ref_lon_deg):
    """Rotate an ECEF delta vector into local East/North/Up at ref_lat/lon."""
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def enu_offset(fiducial_lat, fiducial_lon, fiducial_height, lat, lon, height):
    """East/North/Up/3D offset (meters) of (lat,lon,height) relative to the
    fiducial point."""
    fx, fy, fz = lla_to_ecef(fiducial_lat, fiducial_lon, fiducial_height)
    x, y, z = lla_to_ecef(lat, lon, height)
    dx, dy, dz = x - fx, y - fy, z - fz
    e, n, u = ecef_delta_to_enu(dx, dy, dz, fiducial_lat, fiducial_lon)
    dist_3d = math.sqrt(e * e + n * n + u * u)
    dist_2d = math.sqrt(e * e + n * n)
    return {"east_m": e, "north_m": n, "up_m": u, "dist_2d_m": dist_2d, "dist_3d_m": dist_3d}
