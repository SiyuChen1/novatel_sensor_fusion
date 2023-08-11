import numpy as np


# TODO
def wrap_longitude(lon):
    if lon > 180:
        term = lon % (2 * 180)
        lon = term - 2 * 180 * np.fix(term / 180)
    return lon


def wrap_latitude(lat, lon):
    flat = np.abs(lat)
    if flat > 180:
        lat = (lat + 180) % 360 - 180
        flat = np.abs(lat)
    if flat > 90:
        flat = np.abs(lat)
        lon = lon + 180
        lat = np.sign(lat )


def lla2ecef(latitude, longitude, altitude, radius=6378137, f=1/298.257223563, degrees=False):
    """
    Convert lla to ecef coordinate system.

    https://de.mathworks.com/help/aeroblks/llatoecefposition.html
    """
    if degrees:
        latitude = latitude * np.pi / 180
        longitude = longitude * np.pi / 180

    # https://de.mathworks.com/help/aerotbx/ug/lla2ecef.html
    # lambda_s = np.arctan(np.square(1 - f) * np.tan(latitude))
    # r_s = radius / np.sqrt(1 + (1 / np.square(1 - f) - 1) * np.sin(lambda_s))
    # x = (r_s * np.cos(lambda_s) + altitude * np.cos(latitude)) * np.cos(longitude)
    # y = (r_s * np.cos(lambda_s) + altitude * np.cos(latitude)) * np.sin(longitude)
    # z = r_s * np.sin(lambda_s) + altitude * np.sin(latitude)

    e2 = f * (2 - f)
    N = radius / np.sqrt(1 - e2 * np.square(np.sin(latitude)))
    rho = (N + altitude) * np.cos(latitude)
    z = (N * (1 - e2) + altitude) * np.sin(latitude)
    x = rho * np.cos(longitude)
    y = rho * np.sin(longitude)

    return np.array([x, y, z])


def ecef2enu(lat_ref, lon_ref, alt_ref, xyz, degrees=False):
    if degrees:
        lat_ref = lat_ref * np.pi / 180
        lon_ref = lon_ref * np.pi / 180
    sla = np.sin(lat_ref)
    cla = np.cos(lat_ref)
    slo = np.sin(lon_ref)
    clo = np.cos(lon_ref)
    xyz_ref = lla2ecef(lat_ref, lon_ref, alt_ref, degrees=False)
    xyz_dif = xyz - xyz_ref
    rotation = np.array([[-slo, clo, 0], [-clo * sla, -slo * sla, cla],
                         [cla * clo, slo * cla, sla]])
    return rotation @ xyz_dif


def lla2ecef2enu(latitude, longitude, altitude, lat_ref, lon_ref,
                 alt_ref, radius=6378137, f=1/298.257223563, degrees=False):
    """

    This function converts lla to ecef and then to enu,
    but the transformation is not accurate
    """
    if degrees:
        latitude = latitude * np.pi / 180
        longitude = longitude * np.pi / 180
        lat_ref = lat_ref * np.pi / 180
        lon_ref = lon_ref * np.pi / 180
    xyz = lla2ecef(latitude, longitude, altitude, radius=radius, f=f, degrees=False)
    return ecef2enu(lat_ref, lon_ref, alt_ref, xyz, degrees=False)


def lla2ned(latitude, longitude, altitude, lat_ref, lon_ref,
            alt_ref, radius=6378137, f=1/298.257223563, degrees=False):
    if degrees:
        latitude = latitude * np.pi / 180
        longitude = longitude * np.pi / 180
        lat_ref = lat_ref * np.pi / 180
        lon_ref = lon_ref * np.pi / 180

    dlat = latitude - lat_ref
    dlon = longitude - lon_ref
    e2 = f * (2 - f)
    rn = radius / np.sqrt(1 - e2 * np.sin(lat_ref) * np.sin(lat_ref))
    rm = rn * (1 - e2) / (1 - e2 * np.sin(lat_ref) * np.sin(lat_ref))
    dnorth = dlat / np.arctan2(1, rm)
    deast = dlon / np.arctan2(1, rn * np.cos(lat_ref))
    ned = np.array([dnorth, deast, - altitude + alt_ref])
    return ned


def lla2enu(latitude, longitude, altitude, lat_ref, lon_ref,
            alt_ref, radius=6378137, f=1/298.257223563, degrees=False):
    if degrees:
        latitude = latitude * np.pi / 180
        longitude = longitude * np.pi / 180
        lat_ref = lat_ref * np.pi / 180
        lon_ref = lon_ref * np.pi / 180
    ned = lla2ned(latitude, longitude, altitude, lat_ref, lon_ref, alt_ref,
                  radius, f, degrees=False)
    return np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]) @ ned


def enu2lla(east, north, up, lat_ref, lon_ref, alt_ref,
            radius=6378137, f=1/298.257223563, degrees=False):
    if degrees:
        lat_ref = lat_ref * np.pi / 180
        lon_ref = lon_ref * np.pi / 180
    alt = up + alt_ref
    e2 = f * (2 - f)
    rn = radius / np.sqrt(1 - e2 * np.sin(lat_ref) * np.sin(lat_ref))
    rm = rn * (1 - e2) / (1 - e2 * np.sin(lat_ref) * np.sin(lat_ref))

    dlat = north * np.arctan2(1, rm)
    dlon = east * np.arctan2(1, rn * np.cos(lat_ref))
    lat = dlat + lat_ref
    lon = dlon + lon_ref
    lla = 180 / np.pi * np.array([lat, lon, 0])
    lla[2] = alt
    lla[1] = wrap_longitude(lla[1])
    return lla
