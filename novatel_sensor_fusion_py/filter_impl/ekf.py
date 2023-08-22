import numpy as np
from scipy.spatial.transform import Rotation


def repair_quaternion(q):
    q = 1 / np.sqrt(np.sum(np.square(q))) * q
    if q[0] < 0:
        return -q
    return q


class ExtendedKalmanFilter16States:
    """

    Implementation based on
    https://de.mathworks.com/help/fusion/ug/imu-and-gps-fusion-for-inertial-navigation.html
    """
    def __init__(self, imu_sample_rate, gyro_bias_instability,
                 accelerometer_bias_instability, angular_random_walk,
                 velocity_random_walk, init_state=None,
                 init_state_covariance=None, other_additive_noise=1e-9):
        self.imu_sample_rate = imu_sample_rate

        self.state = np.zeros(16)
        if init_state is not None:
            self.state = init_state

        self.state_covariance = 1e-9 * np.eye(16)
        if init_state_covariance is not None:
            self.state_covariance = init_state_covariance

        # convert bias_instability with unit degree per hour into gyro bias noise with unit (rad/s)^2
        self.gyro_bias_noise = np.square(gyro_bias_instability / 180 * np.pi / 3600)

        # convert accelerometer_bias_noise with unit milli g into (m/s^2)^2
        self.accelerometer_bias_noise = np.square(accelerometer_bias_instability / 1000 * 9.81)

        # convert angular_random_walk with unit degree per sqrt(hour) into gyro noise with unit (rad/s)^2
        # 1. firstly degree per sqrt(hour) into rad^2 per hour
        # secondly converts into psd (rad^2 per s), power spectral density
        # unit of psd should be (rad/s)^2 per Hz which is equivalent to rad^s per s
        gyro_psd = (angular_random_walk * np.pi / 180) ** 2 / 3600

        # 2. gyro noise is the noise density multiplied by the sampling frequency
        self.gyro_noise = gyro_psd * imu_sample_rate

        # (VRW) velocity random walk with the unit m/s/sqrt(hour)
        # 1. from VRW to (m/s)^2 per hour to (m/s)^2 per s
        # since (m/s)^2 per s is (m/s^2)^2 per Hz
        # which is exactly the psd of accelerometer noise
        accel_psd = velocity_random_walk ** 2 / 3600

        # 2. accelerometer noise is the noise density multiplied by the sampling rate
        self.accelerometer_noise = accel_psd * imu_sample_rate

        self.other_additive_noise = other_additive_noise
        self.prev_gyro = None
        self.prev_accel = None

        self.dvel = None
        self.dangle = None

    def get_state(self):
        return self.state

    def get_state_covariance(self):
        return self.state_covariance

    def additive_process_noise(self):
        scale = 0.5 * 1 / (self.imu_sample_rate ** 2)
        dangle_bias_sigma = scale * self.gyro_bias_noise
        dvel_bias_sigma = scale * self.accelerometer_bias_noise
        diag_ele = np.zeros(16)
        diag_ele[0:10] = self.other_additive_noise
        diag_ele[10:13] = dangle_bias_sigma
        diag_ele[13:16] = dvel_bias_sigma
        return np.diag(diag_ele)

    def imu_gyro_accel_noise(self):
        scale = 0.5 * 1 / (self.imu_sample_rate ** 2)
        w = np.zeros(6)
        w[0:3] = scale * self.gyro_noise
        w[3:6] = scale * self.accelerometer_noise
        return w

    def state_transition_jacobian(self, dt):
        qw = self.state[0]
        qx = self.state[1]
        qy = self.state[2]
        qz = self.state[3]
        # pe = self.state[4]
        # pn = self.state[5]
        # pu = self.state[6]
        # ve = self.state[7]
        # vn = self.state[8]
        # vu = self.state[9]
        dthetax_b = self.state[10]
        dthetay_b = self.state[11]
        dthetaz_b = self.state[12]
        dvx_b = self.state[13]
        dvy_b = self.state[14]
        dvz_b = self.state[15]
        
        dthetax = self.dangle[0]
        dthetay = self.dangle[1]
        dthetaz = self.dangle[2]
        dvx = self.dvel[0]
        dvy = self.dvel[1]
        dvz = self.dvel[2]

        dfdx = np.zeros((16, 16))
        dfdx[0, 0] = 1
        dfdx[0, 1] = - (dthetax - dthetax_b) / 2
        dfdx[0, 2] = - (dthetay - dthetay_b) / 2
        dfdx[0, 3] = - (dthetaz - dthetaz_b) / 2
        dfdx[0, 10] = qx / 2
        dfdx[0, 11] = qy / 2
        dfdx[0, 12] = qz / 2

        dfdx[1, 0] = (dthetax - dthetax_b) / 2
        dfdx[1, 1] = 1
        dfdx[1, 2] = - (dthetaz - dthetaz_b) / 2
        dfdx[1, 3] = (dthetay - dthetay_b) / 2
        dfdx[1, 10] = - qw / 2
        dfdx[1, 11] = - qz / 2
        dfdx[1, 12] = qy / 2

        dfdx[2, 0] = (dthetay - dthetay_b) / 2
        dfdx[2, 1] = (dthetaz - dthetaz_b) / 2
        dfdx[2, 2] = 1
        dfdx[2, 3] = - (dthetax - dthetax_b) / 2
        dfdx[2, 10] = qz / 2
        dfdx[2, 11] = - qw / 2
        dfdx[2, 12] = - qx / 2

        dfdx[3, 0] = (dthetaz - dthetaz_b) / 2
        dfdx[3, 1] = - (dthetay - dthetay_b) / 2
        dfdx[3, 2] = (dthetax - dthetax_b) / 2
        dfdx[3, 3] = 1
        dfdx[3, 10] = - qy / 2
        dfdx[3, 11] = qx / 2
        dfdx[3, 12] = - qw / 2

        dfdx[4, 4] = 1
        dfdx[4, 7] = dt
        dfdx[5, 5] = 1
        dfdx[5, 8] = dt
        dfdx[6, 6] = 1
        dfdx[6, 9] = dt

        dfdx[7, 0] = 2 * qw * (dvx - dvx_b) - 2 * qz * (dvy - dvy_b) + 2 * qy * (dvz - dvz_b)
        dfdx[7, 1] = 2 * qx * (dvx - dvx_b) + 2 * qy * (dvy - dvy_b) + 2 * qz * (dvz - dvz_b)
        dfdx[7, 2] = -2 * qy * (dvx - dvx_b) + 2 * qx * (dvy - dvy_b) + 2 * qw * (dvz - dvz_b)
        dfdx[7, 3] = - 2 * qz * (dvx - dvx_b) - 2 * qw * (dvy - dvy_b) + 2 * qx * (dvz - dvz_b)
        dfdx[7, 7] = 1
        dfdx[7, 13] = - (qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2)
        dfdx[7, 14] = 2 * qw * qz - 2 * qx * qy
        dfdx[7, 15] = - 2 * (qw * qy + qx * qz)

        dfdx[8, 0] = 2 * qw * (dvy - dvy_b) + 2 * qz * (dvx - dvx_b) - 2 * qx * (dvz - dvz_b)
        dfdx[8, 1] = -2 * qx * (dvy - dvy_b) + 2 * qy * (dvx - dvx_b) - 2 * qw * (dvz - dvz_b)
        dfdx[8, 2] = 2 * qy * (dvy - dvy_b) + 2 * qx * (dvx - dvx_b) + 2 * qz * (dvz - dvz_b)
        dfdx[8, 3] = -2 * qz * (dvy - dvy_b) + 2 * qw * (dvx - dvx_b) + 2 * qy * (dvz - dvz_b)
        dfdx[8, 8] = 1
        dfdx[8, 13] = -2 * qx * qy - 2 * qw * qz
        dfdx[8, 14] = - (qw ** 2 - qx ** 2 + qy ** 2 - qz ** 2)
        dfdx[8, 15] = -2 * qy * qz + 2 * qw * qx

        dfdx[9, 0] = 2 * qw * (dvz - dvz_b) - 2 * qy * (dvx - dvx_b) + 2 * qx * (dvy - dvy_b)
        dfdx[9, 1] = -2 * qx * (dvz - dvz_b) + 2 * qz * (dvx - dvx_b) + 2 * qw * (dvy - dvy_b)
        dfdx[9, 2] = -2 * qy * (dvz - dvz_b) - 2 * qw * (dvx - dvx_b) + 2 * qz * (dvy - dvy_b)
        dfdx[9, 3] = 2 * qz * (dvz - dvz_b) + 2 * qx * (dvx - dvx_b) + 2 * qy * (dvy - dvy_b)
        dfdx[9, 9] = 1
        dfdx[8, 13] = 2 * qw * qy - 2 * qx * qz
        dfdx[8, 14] = -2 * qw * qx - 2 * qy * qz
        dfdx[8, 15] = - (qw ** 2 - qx ** 2 - qy ** 2 + qz ** 2)

        dfdx[10, 10] = 1
        dfdx[11, 11] = 1
        dfdx[12, 12] = 1
        dfdx[13, 13] = 1
        dfdx[14, 14] = 1
        dfdx[15, 15] = 1
        return dfdx

    def imu_predict(self, accel, gyro, dt, trapezoidal_integration=True):
        qw = self.state[0]
        qx = self.state[1]
        qy = self.state[2]
        qz = self.state[3]
        pe = self.state[4]
        pn = self.state[5]
        pu = self.state[6]
        ve = self.state[7]
        vn = self.state[8]
        vu = self.state[9]
        dthetax_b = self.state[10]
        dthetay_b = self.state[11]
        dthetaz_b = self.state[12]
        dvx_b = self.state[13]
        dvy_b = self.state[14]
        dvz_b = self.state[15]

        p = np.array([pe, pn, pu])
        v = np.array([ve, vn, vu])
        v_b = np.array([dvx_b, dvy_b, dvz_b])
        a_b = np.array([dthetax_b, dthetay_b, dthetaz_b])

        if trapezoidal_integration:
            if self.prev_gyro is None:
                self.prev_gyro = gyro
            if self.prev_accel is None:
                self.prev_accel = accel
            dvel = 0.5 * dt * (self.prev_accel + accel)
            dangle = 0.5 * dt * (self.prev_gyro + gyro)
            self.prev_accel = accel
            self.prev_gyro = gyro
        else:
            dvel = accel * dt
            dangle = gyro * dt
        
        self.dvel = dvel
        self.dangle = dangle

        # R = np.zeros((3, 3))
        # R[0, 0] = qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2
        # R[0, 1] = 2 * (qx * qy - qw * qz)
        # R[0, 2] = 2 * (qw * qy + qx * qz)
        # R[1, 0] = 2 * (qx * qy + qw * qz)
        # R[1, 1] = qw ** 2 - qx ** 2 + qy ** 2 - qz ** 2
        # R[1, 2] = 2 * (qy * qz - qw * qx)
        # R[2, 0] = 2 * (qx * qz - qw * qy)
        # R[2, 1] = 2 * (qw * qx + qy * qz)
        # R[2, 2] = qw ** 2 - qx ** 2 - qy ** 2 + qz ** 2
        cur_rotation = Rotation.from_quat([qx, qy, qz, qw])

        next_state = np.zeros(self.state.shape)
        next_state[4:7] = p + dt * v
        next_state[7:10] = v + cur_rotation.as_matrix() @ np.array(dvel - v_b)

        rotation_delta = Rotation.from_rotvec(dangle - a_b)
        update_rotation = rotation_delta * cur_rotation
        q_update = update_rotation.as_quat()
        q = np.array([q_update[3], q_update[0],
                      q_update[1], q_update[2]])
        q_repair = repair_quaternion(q)

        next_state[0:4] = q_repair

        next_state[10:16] = self.state[10:16]
        self.set_state(next_state)

    def control_input_jacobian(self):
        qw = self.state[0]
        qx = self.state[1]
        qy = self.state[2]
        qz = self.state[3]
        dldx = np.zeros((16, 6))

        R = np.zeros((3, 3))
        R[0, 0] = qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2
        R[0, 1] = 2 * (qx * qy - qw * qz)
        R[0, 2] = 2 * (qw * qy + qx * qz)
        R[1, 0] = 2 * (qx * qy + qw * qz)
        R[1, 1] = qw ** 2 - qx ** 2 + qy ** 2 - qz ** 2
        R[1, 2] = 2 * (qy * qz - qw * qx)
        R[2, 0] = 2 * (qx * qz - qw * qy)
        R[2, 1] = 2 * (qw * qx + qy * qz)
        R[2, 2] = qw ** 2 - qx ** 2 - qy ** 2 + qz ** 2

        dldx[0, 0] = - qx / 2
        dldx[0, 1] = - qy / 2
        dldx[0, 2] = - qz / 2
        dldx[1, 0] = qw / 2
        dldx[1, 1] = qz / 2
        dldx[1, 2] = - qy / 2
        dldx[2, 0] = - qz / 2
        dldx[2, 1] = qw / 2
        dldx[2, 2] = qx / 2
        dldx[3, 0] = qy / 2
        dldx[3, 1] = - qx / 2
        dldx[3, 2] = qw / 2

        dldx[7:10, 3:6] = R
        return dldx

    def control_input_noise(self):
        L = self.control_input_jacobian()
        return L @ np.diag(self.imu_gyro_accel_noise()) @ np.transpose(L)

    def compute_next_state_covariance(self, dt):
        dfdx = self.state_transition_jacobian(dt)
        next_state_covariance = dfdx @ self.state_covariance @ np.transpose(dfdx) + \
            self.control_input_noise() + self.additive_process_noise()
        return next_state_covariance

    def fuse_gps(self, gps_position, gps_velocity, gps_position_std, gps_velocity_std):
        # gps position is given in ENU coordinate system
        measure_noise_w = np.zeros(6)
        measure_noise_w[0:3] = gps_position_std
        measure_noise_w[3:6] = gps_velocity_std
        measure_cov = np.diag(measure_noise_w)

        z = np.array([gps_position[0], gps_position[1], gps_position[2],
                      gps_velocity[0], gps_velocity[1], gps_velocity[2]])
        h = self.measure_fun()
        dhdx = self.measure_jacobian_fun()

        p = self.get_state_covariance()
        innov_covariance = dhdx @ p @ np.transpose(dhdx) + measure_cov
        innov = z - h

        w = p @ np.transpose(dhdx) @ np.linalg.inv(innov_covariance)
        state = self.get_state() + w @ innov
        p = p - w @ dhdx @ p

        q = state[0:4]
        q = repair_quaternion(q)
        state[0:4] = q

        self.set_state(state)
        self.set_state_covariance(p)

    def measure_fun(self):
        return self.state[4:10]

    def measure_jacobian_fun(self):
        dhdx = np.zeros((6, 16))
        dhdx[0, 4] = 1
        dhdx[1, 5] = 1
        dhdx[2, 6] = 1
        dhdx[3, 7] = 1
        dhdx[4, 8] = 1
        dhdx[5, 9] = 1
        return dhdx

    def set_state(self, state):
        self.state = state

    def set_state_covariance(self, state_covariance):
        self.state_covariance = state_covariance
