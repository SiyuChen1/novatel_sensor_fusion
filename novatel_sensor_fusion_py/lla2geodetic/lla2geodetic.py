import numpy as np


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


def lla2enu(latitude, longitude, altitude, lat_ref, lon_ref,
            alt_ref, radius=6378137, f=1/298.257223563, degrees=False):
    if degrees:
        latitude = latitude * np.pi / 180
        longitude = longitude * np.pi / 180
        lat_ref = lat_ref * np.pi / 180
        lon_ref = lon_ref * np.pi / 180
    xyz = lla2ecef(latitude, longitude, altitude, radius=radius, f=f, degrees=False)
    return ecef2enu(lat_ref, lon_ref, alt_ref, xyz, degrees=False)
