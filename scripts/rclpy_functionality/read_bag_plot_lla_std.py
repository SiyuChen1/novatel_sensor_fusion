import argparse
import matplotlib.pyplot as plt
import numpy as np

from novatel_sensor_fusion.msg import NavSatExtended
from rclpy.serialization import deserialize_message
import rosbag2_py


parser = argparse.ArgumentParser(description="Visualizing LLA Standard Deviation")
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

bestgnss = []

# Read messages
while reader.has_next():
    (topic, msg_bytes, t) = reader.read_next()
    ts = t / 1e9
    match topic:
        case '/bestgnss':
            msg = deserialize_message(msg_bytes, NavSatExtended)
            bestgnss.append([
                ts, msg.position_covariance[0],
                msg.position_covariance[4], msg.position_covariance[8]
            ])

bestgnss = np.array(bestgnss)
x_id = list(range(bestgnss.shape[0]))

plt.plot(x_id, bestgnss[:, 1], label='latitude')
plt.plot(x_id, bestgnss[:, 2], label='longitude')
plt.plot(x_id, bestgnss[:, 3], label='altitude')
plt.title('standard deviation of gnss measurements')
plt.xlabel('measurement count')
plt.ylabel('std in m')
plt.legend()
plt.show()
