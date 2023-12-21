import numpy as np


freq = 100

# gyro
# gyro_bias_instability = 0.45 deg/hour
# convert it into rad/s
gyro_bias_instability = 0.45 / 180 * np.pi / 3600
print('gyro_bias_instability rad/s', gyro_bias_instability)

# angular (angle) random walk is equal to gyroscope noise density
# see https://github.com/ori-drs/allan_variance_ros/blob/master/scripts/analysis.py
# line 209-220
# gyro_random_walk = 0.06 deg/sqrt(hour)
# convert it into rad/sqrt(s) which follows orb slam3 convention
gyro_random_walk = 0.06 / 180 * np.pi / np.sqrt(3600)
gyroscope_noise_density = gyro_random_walk
print('gyroscope_noise_density rad/sqrt(s)', gyroscope_noise_density)

# accel
# accel_bias_instability = 0.075mg
# convert it into m/s^2
accel_bias_instability = 0.075 * 1e-3 * 9.8067
print('accel_bias_instability m/s^2', accel_bias_instability)

# velocity random walk is equal to noise density of accelerometer
# see https://github.com/ori-drs/allan_variance_ros/blob/master/scripts/analysis.py
# line 120-131
# accel_random_walk = 0.06 m/s/sqrt(hour)
# convert it into m/s/sqrt(s) which follows orb slam3 convention
accel_random_walk = 0.06 / np.sqrt(3600)
accel_noise_density = accel_random_walk
print('accel_noise_density m/s^1.5', accel_noise_density)

# random walk used in orb slam3 means different
# for gyroscope, it should be rate random walk, but not given in the datasheet
# for accelerometer, it should be acceleration random walk, not given either

# both can be calibrated using https://github.com/ori-drs/allan_variance_ros.git
