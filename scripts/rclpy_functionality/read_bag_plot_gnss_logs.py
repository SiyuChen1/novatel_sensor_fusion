import argparse
import matplotlib.pyplot as plt
import numpy as np

from novatel_sensor_fusion.msg import NavRMC, NavSat, NavSatExtended
from novatel_sensor_fusion_py.lla2geodetic.lla2geodetic import lla2enu
import novatel_sensor_fusion_py.ultilies.color as color
from novatel_sensor_fusion_py.ultilies.trajectory_similarity import compute_rmse_between_trajectories
from rclpy.serialization import deserialize_message
import rosbag2_py


parser = argparse.ArgumentParser(description="Visualizing LLA")
parser.add_argument("--bag-file-path", type=str,
                    default='/home/siyuchen/catkin_ws/imu_raw_data_bag_recorder/imu_raw_data_bag_recorder_0.mcap')

# Path to the bag file
# bag_file_path = '/home/siyuchen/catkin_ws/' \
#                 'imu_raw_data_bag_recorder/imu_raw_data_bag_recorder_0.mcap'
# Parse the arguments
args = parser.parse_args()
bag_file_path = args.bag_file_path

# Create a reader
reader = rosbag2_py.SequentialReader()

# Open the bag file
storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='mcap')
reader.open(storage_options, rosbag2_py.ConverterOptions())

# # Iterate through messages
# for topic_metadata in reader.get_all_topics_and_types():
#     print(f'topic: {topic_metadata.name}, type: {topic_metadata.type}')

gprmc = []
gpgga = []
gpggalong = []
best = []
bestgnss = []
bestutm = []
bestxyz = []

# Read messages
while reader.has_next():
    (topic, msg_bytes, t) = reader.read_next()
    ts = t / 1e9
    match topic:
        case '/gprmc':
            msg = deserialize_message(msg_bytes, NavRMC)
            # convert time in nanosecs into secs
            gprmc.append([ts, msg.latitude, msg.longitude])
        case '/gpgga':
            msg = deserialize_message(msg_bytes, NavSat)
            gpgga.append([ts, msg.latitude, msg.longitude, msg.altitude])
        case '/gpggalong':
            msg = deserialize_message(msg_bytes, NavSat)
            gpggalong.append([ts, msg.latitude, msg.longitude, msg.altitude])
        case '/best':
            msg = deserialize_message(msg_bytes, NavSatExtended)
            best.append([ts, msg.latitude, msg.longitude, msg.altitude])
        case '/bestgnss':
            msg = deserialize_message(msg_bytes, NavSatExtended)
            bestgnss.append([ts, msg.latitude, msg.longitude, msg.altitude])

# Close the reader
del reader

start_ts = min(gprmc[0][0], gpgga[0][0], gpggalong[0][0], best[0][0], bestgnss[0][0])

gprmc = np.array(gprmc)
gprmc[:, 0] = gprmc[:, 0] - start_ts
gpgga = np.array(gpgga)
gpgga[:, 0] = gpgga[:, 0] - start_ts
gpggalong = np.array(gpggalong)
gpggalong[:, 0] = gpggalong[:, 0] - start_ts
best = np.array(best)
best[:, 0] = best[:, 0] - start_ts
bestgnss = np.array(bestgnss)
bestgnss[:, 0] = bestgnss[:, 0] - start_ts

fig, axs = plt.subplots(3, 1)
axs[0].plot(gprmc[:, 0], gprmc[:, 1], label='GPRMC')
axs[0].plot(gpgga[:, 0], gpgga[:, 1], label='GPGGA')
axs[0].plot(gpggalong[:, 0], gpggalong[:, 1], label='GPGGALong')
axs[0].plot(best[:, 0], best[:, 1], label='BEST')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Latitude in degree')
axs[0].ticklabel_format(useOffset=False)

axs[1].plot(gprmc[:, 0], gprmc[:, 2], label='GPRMC')
axs[1].plot(gpgga[:, 0], gpgga[:, 2], label='GPGGA')
axs[1].plot(gpggalong[:, 0], gpggalong[:, 2], label='GPGGALong')
axs[1].plot(best[:, 0], best[:, 2], label='BEST')
axs[1].ticklabel_format(useOffset=False)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Longitude in degree')

lat_ref = gpggalong[0, 1]
lon_ref = gpggalong[0, 2]
alt_ref = gpggalong[0, 3]
# print('ref lla', lat_ref, lon_ref, alt_ref)
# print(np.min(best[:, 1]))
# print(np.max(best[:, 1]))

gprmc_enu = lla2enu(
    gprmc[:, 1], gprmc[:, 2], np.zeros(gprmc.shape[0]),
    lat_ref, lon_ref, alt_ref, degrees=True)

gpgga_enu = lla2enu(
    gpgga[:, 1], gpgga[:, 2], gpgga[:, 3],
    lat_ref, lon_ref, alt_ref, degrees=True)

gpggalong_enu = lla2enu(
    gpggalong[:, 1], gpggalong[:, 2], gpggalong[:, 3],
    lat_ref, lon_ref, alt_ref, degrees=True)

best_enu = lla2enu(
    best[:, 1], best[:, 2], best[:, 3],
    lat_ref, lon_ref, alt_ref, degrees=True
)

bestgnss_enu = lla2enu(
    bestgnss[:, 1], bestgnss[:, 2], bestgnss[:, 3],
    lat_ref, lon_ref, alt_ref, degrees=True
)

print(gprmc_enu.shape)
print(gpggalong_enu.shape)
print(best_enu.shape)

axs[2].plot(gprmc_enu[0, :], gprmc_enu[1, :], '+', label='GPRMC')
axs[2].plot(gpgga_enu[0, :], gpgga_enu[1, :], '*', label='GPGGA')
axs[2].plot(gpggalong_enu[0, :], gpggalong_enu[1, :], 'x', label='GPGGALong')
axs[2].set_xlabel('East m')
axs[2].set_ylabel('North m')
axs[2].ticklabel_format(useOffset=False)
plt.legend()
plt.show()

print('distance between gpgga and gpggalong in meter',
      compute_rmse_between_trajectories(t_a=gpggalong[:, 0],
                                        t_b=gpgga[:, 0],
                                        trajectory_a=gpggalong_enu,
                                        trajectory_b=gpgga_enu
                                        )
      )

print('distance between gprmc and gpggalong in meter',
      compute_rmse_between_trajectories(t_a=gpggalong[:, 0],
                                        t_b=gprmc[:, 0],
                                        trajectory_a=gpggalong_enu[0:2, :],
                                        trajectory_b=gprmc_enu[0:2, :]
                                        )
      )

print('distance between best and gpggalong in meter',
      compute_rmse_between_trajectories(t_a=gpggalong[:, 0],
                                        t_b=best[:, 0],
                                        trajectory_a=gpggalong_enu,
                                        trajectory_b=best_enu
                                        )
      )

print('distance between bestgnss and gpggalong in meter',
      compute_rmse_between_trajectories(t_a=gpggalong[:, 0],
                                        t_b=bestgnss[:, 0],
                                        trajectory_a=gpggalong_enu,
                                        trajectory_b=bestgnss_enu
                                        )
      )


print('distance between bestgnss and best in meter',
      compute_rmse_between_trajectories(t_a=best[:, 0],
                                        t_b=bestgnss[:, 0],
                                        trajectory_a=best_enu,
                                        trajectory_b=bestgnss_enu
                                        )
      )

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.plot(gpggalong_enu[0, :], gpggalong_enu[1, :], gpggalong_enu[2, :], color.DARK_RED, label='gpggalong')
ax.plot(best_enu[0, :], best_enu[1, :], best_enu[2, :], color.INDIGO, label='best')
ax.plot(bestgnss_enu[0, :], bestgnss_enu[1, :], bestgnss_enu[2, :], color.YELLOW, label='bestgnss')
ax.set_xlabel('East')
ax.set_ylabel('North')
ax.set_zlabel('Up')
ax.legend()
# ax.plot3D(bestgnss_enu[0, :], bestgnss_enu[1, :], bestgnss_enu[2, :], 'red')

plt.show()

