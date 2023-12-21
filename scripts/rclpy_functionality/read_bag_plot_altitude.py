import matplotlib.pyplot as plt
import numpy as np

from novatel_sensor_fusion.msg import NavRMC, NavSat, NavSatExtended
from rclpy.serialization import deserialize_message
import rosbag2_py


# Path to the bag file
bag_file_path = '/home/siyuchen/catkin_ws/' \
                '20230907_Laurensberg/20230907_Laurensberg_0.mcap'

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
axs[0].legend()

axs[0].ticklabel_format(useOffset=False)
axs[1].plot(gprmc[:, 0], gprmc[:, 2], label='GPRMC')
axs[1].plot(gpgga[:, 0], gpgga[:, 2], label='GPGGA')
axs[1].plot(gpggalong[:, 0], gpggalong[:, 2], label='GPGGALong')
axs[1].plot(best[:, 0], best[:, 2], label='BEST')
axs[1].ticklabel_format(useOffset=False)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Longitude in degree')
axs[1].legend()


axs[2].plot(gpgga[:, 0], gpgga[:, 3], label='GPGGA')
axs[2].plot(gpggalong[:, 0], gpggalong[:, 3], label='GPGGALong')
axs[2].plot(best[:, 0], best[:, 3], label='BEST')
axs[2].plot(bestgnss[:, 0], bestgnss[:, 3], label='BESTGNSS')
axs[2].ticklabel_format(useOffset=False)
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Altitude in meter')

plt.legend()
plt.show()

