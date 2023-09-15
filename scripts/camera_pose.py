from geometry_msgs.msg import TransformStamped
from novatel_sensor_fusion_py.raw_data_to_bag.data2rosmsg import gps_time_to_ros_time
import numpy as np
from scipy.spatial.transform import Rotation


def get_camera_pose_start_end_id(camera_pose_path):
    camera_pose_path = camera_pose_path.split('/')[-1]
    camera_pose_path = camera_pose_path.split('.')[0]
    ids = camera_pose_path.split('_')
    return {'start': int(ids[1]), 'end': int(ids[2])}


time_stamps = []
with open('../slam/Stereocam_20230703_140222.log', 'r') as file:
    for line in file:
        gps_sec = float(line.split(' ')[0])
        time_stamps.append(gps_sec)

camera_pose_file_path = '../slam/CameraTrajectory_1685_2178.txt'
ids = get_camera_pose_start_end_id(camera_pose_file_path)
gps_week = 2269

transforms = []
with open(camera_pose_file_path, 'r') as file:
    for line in file:
        transform = list(map(float, line.split(' ')))
        transform.extend([0, 0, 0, 1])
        transforms.append(transform)

transforms = np.array(transforms).reshape(-1, 4, 4)
print(transforms.shape)

for i in range(transforms.shape[0]):
    index = i + ids['start'] + 1
    ros_time = gps_time_to_ros_time(int(gps_week), float(time_stamps[index]))
    current_transform = transforms[i, :, :]
    msg = TransformStamped()
    msg.header.stamp = ros_time
    msg.header.frame_id = 'base'
    msg.child_frame_id = 'left_camera'
    msg.transform.translation.x = current_transform[0, 3]
    msg.transform.translation.y = current_transform[1, 3]
    msg.transform.translation.z = current_transform[2, 3]
    r = Rotation.from_matrix(current_transform[0:3, 0:3])
    msg.transform.rotation.w = r.as_quat()[3]
    msg.transform.rotation.x = r.as_quat()[0]
    msg.transform.rotation.y = r.as_quat()[1]
    msg.transform.rotation.z = r.as_quat()[2]
