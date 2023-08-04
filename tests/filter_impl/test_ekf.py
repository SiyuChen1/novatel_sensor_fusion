from novatel_sensor_fusion_py.filter_impl.ekf import ExtendedKalmanFilter16States
import numpy as np
import pytest


init_16_states = np.array([0.5251, 0.0513, 0.0147, 0.8493, 2.5541,
                           5.4406, 0.7544, 0, 0, 0, 0.0003, 0.0003,
                           0.0003, 0.0012, 0.0012, 0.0012])
init_16_states_covariance = 1e-5 * np.eye(16)
imu_sample_rate = 160
dt = 1 / imu_sample_rate
gyro_bias_noise = 1.3436e-14
accelerometer_bias_noise = 0.010716
gyro_noise = 0.00016528
accelerometer_noise = 9.7785

additive_process_noise = 1e-6 * np.diag([0.0010, 0.0010, 0.0010, 0.0010, 0.0010,
                                         0.0010, 0.0010, 0.0010, 0.0010, 0.0010,
                                         0.2624e-12, 0.2624e-12, 0.2624e-12,
                                         0.2093, 0.2093, 0.2093])

imu_gyro_accel_noise = np.array([3.22812500e-09, 3.22812500e-09, 3.22812500e-09,
                                 1.90986328e-04, 1.90986328e-04, 1.90986328e-04])


accel = np.array([0.8806, 0.9547, 0.861])
gyro = np.array([0.0491, 0.0578, 0.0535])


@pytest.fixture
def ekf_16_states_impl_1():
    return ExtendedKalmanFilter16States(
        imu_sample_rate=imu_sample_rate,
        gyro_bias_noise=gyro_bias_noise,
        accelerometer_bias_noise=accelerometer_bias_noise,
        gyro_noise=gyro_noise,
        accelerometer_noise=accelerometer_noise)


@pytest.fixture
def ekf_16_states_impl_2():
    return ExtendedKalmanFilter16States(
        imu_sample_rate=imu_sample_rate,
        gyro_bias_noise=gyro_bias_noise,
        accelerometer_bias_noise=accelerometer_bias_noise,
        gyro_noise=gyro_noise,
        accelerometer_noise = accelerometer_noise,
        init_state=init_16_states,
        init_state_covariance=init_16_states_covariance)


def test_ekf_16_states_get_state(ekf_16_states_impl_1, ekf_16_states_impl_2):
    assert np.allclose(ekf_16_states_impl_1.get_state(), np.zeros(16))
    assert np.allclose(ekf_16_states_impl_2.get_state(), init_16_states)


def test_ekf_16_states_get_state_covariance(ekf_16_states_impl_1, ekf_16_states_impl_2):
    assert np.allclose(ekf_16_states_impl_1.get_state_covariance(), 1e-9 * np.eye(16))
    assert np.allclose(ekf_16_states_impl_2.get_state_covariance(), init_16_states_covariance)


def test_ekf_16_states_additive_process_noise(ekf_16_states_impl_1):
    assert np.allclose(ekf_16_states_impl_1.additive_process_noise(), additive_process_noise)


def test_ekf_16_states_imu_gyro_accel_noise(ekf_16_states_impl_1):
    assert np.allclose(ekf_16_states_impl_1.imu_gyro_accel_noise(), imu_gyro_accel_noise)


def test_ekf_16_states_imu_gyro_accel_noise(ekf_16_states_impl_2):
    print(ekf_16_states_impl_2.compute_next_state(accel, gyro, dt, trapezoidal_integration=True))