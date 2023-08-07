from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from gps_time import GPSTime
from novatel_sensor_fusion.msg import NavSatStatusExtended
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
import transforms3d


def gps_time_to_ros_time(gps_week, seconds_in_week):
    current_gps_time = GPSTime(week_number=int(gps_week), time_of_week=float(seconds_in_week))
    # convert gps time to python datetime and then to posix time
    current_posix_time = current_gps_time.to_datetime().timestamp()
    stamp = Time()
    # https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html
    stamp.sec = int((current_posix_time // 1))
    stamp.nanosec = int(1e9 * (current_posix_time % 1))
    return stamp


def gps_time_to_ros_header(gps_week, seconds_in_week, frame_id):
    header = Header()
    # https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html
    header.stamp = gps_time_to_ros_time(gps_week, seconds_in_week)
    header.frame_id = frame_id
    return header


def data_to_gnss_msg(header_, data_, frame_id):
    sol_status = data_[0]
    pos_type = data_[1]
    gps_ref_time_status = header_[4]
    if gps_ref_time_status != 'FINESTEERING':
        # print("gps_ref_time_status", gps_ref_time_status)
        # sys.exit("in BESTGNSSPOS")
        raise ValueError('gps_ref_time_status should be FINESTEERING, but {} is given'.
                         format(gps_ref_time_status))
    if sol_status != 'SOL_COMPUTED':
        # print("sol_status", sol_status)
        # sys.exit("in BESTGNSSPOS")
        return None
    # if pos_type != "SINGLE":
    #     print("pos_type", pos_type)
    #     sys.exit("in BESTGNSSPOS")
    gnss_msg = NavSatStatusExtended()
    gnss_msg.header = gps_time_to_ros_header(header_[5], header_[6], frame_id)
    gnss_msg.sol_status.data = sol_status
    gnss_msg.pos_type.data = pos_type
    gnss_msg.latitude = float(data_[2])
    gnss_msg.longitude = float(data_[3])
    gnss_msg.altitude = float(data_[4])
    gnss_msg.position_covariance[0] = float(data_[7])
    gnss_msg.position_covariance[4] = float(data_[8])
    gnss_msg.position_covariance[8] = float(data_[9])
    gnss_msg.undulation = float(data_[5])
    gnss_msg.nb_sat_tracked = int(data_[13])
    gnss_msg.nb_sat_solution = int(data_[14])
    return gnss_msg


def data_to_tf(header_, data_, translation, base_id, child_id):
    """
    Convert raw data to ros2 tf messages.

    :param header_:
    :param data_:
    :param translation: translation from msg.child_frame_id to msg.header.frame_id
    :param base_id:
    :param child_id:
    :return:
    """
    ins_status = str(data_[6]).split('*')[0]
    if ins_status != 'INS_SOLUTION_GOOD':
        # raise ValueError("INS doesn't work")
        return None

    gps_week = header_[1]
    seconds_after_week = header_[2]

    msg = TransformStamped()
    # i means instantaneous
    msg.header = gps_time_to_ros_header(gps_week, seconds_after_week, base_id)
    msg.child_frame_id = child_id

    w = float(data_[2])
    x = float(data_[3])
    y = float(data_[4])
    z = float(data_[5])
    q = [w, x, y, z]
    r = transforms3d.quaternions.quat2mat(q)
    translation_base_to_child = - r @ translation

    msg.transform.rotation.w = float(data_[2])
    msg.transform.rotation.x = float(data_[3])
    msg.transform.rotation.y = float(data_[4])
    msg.transform.rotation.z = float(data_[5])

    msg.transform.translation.x = translation_base_to_child[0]
    msg.transform.translation.z = translation_base_to_child[1]
    msg.transform.translation.y = translation_base_to_child[2]

    tf_msg = TFMessage()
    tf_msg.transforms = [msg]
    return tf_msg


def data_to_tf_paras(header_, data_, paras):
    """
    Convert raw data to paras used for generating ros2 tf messages.

    :param header_:
    :param data_:
    :param paras: first element is base_id and second element is child_id,
    last element is translation from msg.child_frame_id to msg.header.frame_id,
    :return:
    """
    base_id = paras[0]
    child_id = paras[1]
    translation = paras[2]
    return data_to_tf(header_, data_, translation, base_id, child_id)


def data_to_imu(header_, data_, paras):
    gps_week = header_[1]
    seconds_after_week = header_[2]

    IMU_DATA_RATE = paras[0]
    frame_id = paras[1]

    msg = Imu()
    msg.header = gps_time_to_ros_header(gps_week, seconds_after_week, frame_id)
    imu_data_count = int(data_[0])
    if imu_data_count > 0:
        msg.angular_velocity.x = IMU_DATA_RATE / imu_data_count * float(data_[1])
        msg.angular_velocity.y = IMU_DATA_RATE / imu_data_count * float(data_[2])
        msg.angular_velocity.z = IMU_DATA_RATE / imu_data_count * float(data_[3])
        msg.linear_acceleration.x = IMU_DATA_RATE / imu_data_count * float(data_[4])
        msg.linear_acceleration.y = IMU_DATA_RATE / imu_data_count * float(data_[5])
        msg.linear_acceleration.z = IMU_DATA_RATE / imu_data_count * float(data_[6])
        return msg
    return None
