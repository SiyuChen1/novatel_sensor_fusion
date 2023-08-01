import numpy as np
from scipy.spatial.transform import Rotation


class ExtendedKalmanFilter16States:
    """
    Implementation based on
    https://de.mathworks.com/help/fusion/ug/imu-and-gps-fusion-for-inertial-navigation.html
    """
    def __init__(self, imu_sample_rate, gyro_bias_noise,
                 accelerometer_bias_noise, gyro_noise,
                 accelerometer_noise, init_state=None,
                 init_state_covariance=None, other_additive_noise=1e-9):
        self.imu_sample_rate = imu_sample_rate

        self.state = np.zeros(16)
        if init_state is not None:
            self.state = init_state

        self.state_covariance = 1e-9 * np.eye(16)
        if init_state_covariance is not None:
            self.state_covariance = init_state_covariance

        self.gyro_bias_noise = gyro_bias_noise
        self.accelerometer_bias_noise = accelerometer_bias_noise
        self.gyro_noise = gyro_noise
        self.accelerometer_noise = accelerometer_noise
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
        dfdx[7, 14] = 2 * qw * qz - 2* qx * qy
        dfdx[7, 15] = - 2 * (qw * qy + qx * qz)



    def predict(self, accel, gyro, dt, trapezoidal_integration=True):
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

        next_state[0] = update_rotation.as_quat()[3]
        next_state[1] = update_rotation.as_quat()[0]
        next_state[2] = update_rotation.as_quat()[1]
        next_state[3] = update_rotation.as_quat()[2]
        self.state = next_state


class Car:

    def __init__(self, speed=0):
        self.speed = speed
        self.odometer = 0
        self.time = 0

    def say_state(self):
        print("I'm going {} kph!".format(self.speed))

    def accelerate(self):
        self.speed += 5

    def brake(self):
        if self.speed < 5:
            self.speed = 0
        else:
            self.speed -= 5

    def step(self):
        self.odometer += self.speed
        self.time += 1

    def average_speed(self):
        if self.time != 0:
            return self.odometer / self.time
        else:
            pass