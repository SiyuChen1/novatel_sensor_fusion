from novatel_sensor_fusion_py.lla2geodetic.lla2geodetic import ecef2enu, enu2lla, lla2ecef, lla2enu, lla2ned
import numpy as np


def test_lla2ecef():
    lat = 45
    lon = 78
    alt = 1000
    xyz = lla2ecef(lat, lon, alt, degrees=True)
    xyz_real = 1e6 * np.array([0.9394, 4.4196, 4.4881])
    assert np.allclose(1 / 1e6 * xyz, 1 / 1e6 * xyz_real, 1e-1)

    lat2 = np.pi / 3
    lon2 = np.pi / 6
    alt2 = 120
    xyz2 = lla2ecef(lat2, lon2, alt2, degrees=False)
    xyz2_real = 1e6 * np.array([2.7688, 1.5986, 5.5006])
    assert np.allclose(1 / 1e6 * xyz2, 1 / 1e6 * xyz2_real, 1e-1)


def test_ecef2enu():
    lat_ref = 45.9132
    lon_ref = 36.7484
    alt_ref = 1877.7532

    x = 5507528.9
    y = 4556224.1
    z = 6012.8208
    xyz = np.array([x, y, z])
    enu = ecef2enu(lat_ref, lon_ref, alt_ref, xyz, degrees=True)
    enu_real = np.array([3.556e5, -5.1023e6, -1.3977e6])
    assert np.allclose(1 / 1e5 * enu, 1 / 1e5 * enu_real, 1e-3)


def test_lla2enu():
    lat_ref = 45.9132
    lon_ref = 36.7484
    alt_ref = 1877.7532

    lat = 45.9142
    lon = 36.7494
    alt = 1687.7532
    enu = lla2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref, degrees=True)
    enu_real = np.array([77.5843, 111.1496, -190.0000])
    assert np.allclose(enu, enu_real, 1e-1)


def test_enu2lla():
    lat_ref = 45.9132
    lon_ref = 36.7484
    alt_ref = 1877.7532

    east = 77.5843
    north = 111.1496
    up = -190.0000

    lla_real = np.array([45.9142, 36.7494, 1687.7532])
    lla = enu2lla(east, north, up, lat_ref, lon_ref, alt_ref, degrees=True)
    assert np.allclose(lla, lla_real)

    enu = lla2enu(lla[0], lla[1], lla[2], lat_ref, lon_ref, alt_ref, degrees=True)
    assert np.allclose(enu, np.array([east, north, up]))


def test_lla2ned():
    lat_ref = 45.9132
    lon_ref = 36.7484
    alt_ref = 1877.7532

    lla = np.array([45.9142, 36.7494, 1687.7532])
    ned_real = np.array([111.1496, 77.5843, 190.0000])

    ned = lla2ned(lla[0], lla[1], lla[2], lat_ref, lon_ref, alt_ref, degrees=True)
    assert np.allclose(ned, ned_real)
