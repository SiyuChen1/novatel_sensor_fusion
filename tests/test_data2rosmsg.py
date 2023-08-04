from novatel_sensor_fusion_py.raw_data_to_bag.data2rosmsg import (
    data_to_gnss_msg, data_to_tf, data_to_tf_paras, gps_time_to_ros_header, data_to_imu)
import pytest


def test_gps_time_to_ros_header():
    gps_week = 2269
    seconds_after_week = 129755.020000000
    frame = 'sat'
    header = gps_time_to_ros_header(gps_week, seconds_after_week, frame)
    assert header.frame_id == frame
    assert header.stamp.sec == 1688378555
    assert header.stamp.nanosec / 1e9 == pytest.approx(seconds_after_week % 1, 2e-6)


def test_data_to_gnss_msg():
    raw_data_1 = '#BESTGNSSPOSA,ICOM5,0,64.0,FINESTEERING,2269,129755.200,02000800,c292,16482;' \
               'SOL_COMPUTED,NARROW_INT,50.77766795767,6.07832613239,178.5262,46.6000,WGS84,' \
               '0.0710,0.0614,0.1463,"1907",2.200,0.000,13,13,13,13,00,21,05,' \
               '03*c527bf59'.split(';')
    header_1 = raw_data_1[0].split(',')
    data_1 = raw_data_1[1].split(',')
    frame_id_1 = 'test_1'
    gnss_msg = data_to_gnss_msg(header_1, data_1, frame_id_1)

    assert gnss_msg.sol_status.data == 'SOL_COMPUTED'
    assert gnss_msg.pos_type.data == 'NARROW_INT'
    assert gnss_msg.latitude == pytest.approx(50.77766795767)
    assert gnss_msg.longitude == pytest.approx(6.07832613239)
    assert gnss_msg.altitude == pytest.approx(178.5262)
    assert gnss_msg.undulation == pytest.approx(46.600)
    assert gnss_msg.position_covariance[0] == pytest.approx(0.0710)
    assert gnss_msg.position_covariance[4] == pytest.approx(0.0614)
    assert gnss_msg.position_covariance[8] == pytest.approx(0.1463)
    assert gnss_msg.nb_sat_tracked == 13
    assert gnss_msg.nb_sat_solution == 13
    assert gnss_msg.header.frame_id == frame_id_1
    assert gnss_msg.header.stamp.sec == 1688378555
    assert gnss_msg.header.stamp.nanosec / 1e9 == pytest.approx(129755.200 % 1)

    raw_data_2 = '#BESTGNSSPOSA,ICOM5,0,64.0,FINESTEERING,2269,129755.200,02000800,c292,16482;'\
        'SOL_IMCOMPUTED,NARROW_INT,50.77766795767,6.07832613239,178.5262,'\
        '46.6000,WGS84,0.0710,0.0614,0.1463,"1907",2.200,0.000,13,13,13,13,00,21,05,'\
        '03*c527bf59'.split(';')
    header_2 = raw_data_2[0].split(',')
    data_2 = raw_data_2[1].split(',')
    frame_id_2 = 'test_2'
    gnss_msg_2 = data_to_gnss_msg(header_2, data_2, frame_id_2)
    assert gnss_msg_2 is None

    raw_data_3 = '#BESTGNSSPOSA,ICOM5,0,64.0,UNKNOWN,2269,129755.200,02000800,c292,16482;' \
        'SOL_COMPUTED,NARROW_INT,50.77766795767,6.07832613239,178.5262,46.6000,WGS84,' \
        '0.0710,0.0614,0.1463,"1907",2.200,0.000,13,13,13,13,00,21,05,03*c527bf59'.split(';')
    header_3 = raw_data_3[0].split(',')
    data_3 = raw_data_3[1].split(',')
    frame_id_3 = 'test_3'
    with pytest.raises(ValueError):
        data_to_gnss_msg(header_3, data_3, frame_id_3)


def test_data_to_tf():
    raw_data = '%INSATTQSA,2269,129755.020;2269,129755.020000000,' \
        '0.957748313,0.016255130,0.004181768,-0.287117490,'\
        'INS_SOLUTION_GOOD*aa7f081f'.split(';')
    dx = 0.431
    dy = 0.0
    dz = 0.013
    # translation_imu2ll = [dx, dy, dz]
    translation = [dx, -dy, -dz]
    header_ = raw_data[0].split(',')
    data_ = raw_data[1].split(',')
    base_id = 'local_level_i'
    child_id = 'vehicle'
    tfs = data_to_tf(header_, data_, translation, base_id, child_id)
    msg = tfs.transforms[0]

    assert msg.header.frame_id == base_id
    assert msg.child_frame_id == child_id
    assert msg.transform.rotation.w == pytest.approx(0.957748313)
    assert msg.transform.rotation.x == pytest.approx(0.016255130)
    assert msg.transform.rotation.y == pytest.approx(0.004181768)
    assert msg.transform.rotation.z == pytest.approx(-0.287117490)

    assert msg.transform.translation.x == pytest.approx(-0.3599419168687815)
    assert msg.transform.translation.y == pytest.approx(0.02046812329572577)
    assert msg.transform.translation.z == pytest.approx(0.23654359515653375)


def test_data_to_tf_2():
    raw_data = '%INSATTQSA,2269,129755.020;2269,129755.020000000,' \
               '0.957748313,0.016255130,0.004181768,-0.287117490,INS_INACTIVE*aa7f081f'.split(';')
    translation = [0.431, 0.0, 0.013]
    header_ = raw_data[0].split(',')
    data_ = raw_data[1].split(',')
    base_id = 'local_level_i'
    child_id = 'vehicle'
    msg = data_to_tf(header_, data_, translation, base_id, child_id)
    assert msg is None


def test_data_to_tf_paras():
    raw_data = '%INSATTQSA,2269,129755.020;2269,129755.020000000,'\
        '0.957748313,0.016255130,0.004181768,-0.287117490,'\
        'INS_SOLUTION_GOOD*aa7f081f'.split(';')
    translation = [0.431, 0.0, 0.013]
    header_ = raw_data[0].split(',')
    data_ = raw_data[1].split(',')
    base_id = 'local_level_i'
    child_id = 'vehicle'
    paras = [base_id, child_id, translation]
    tfs = data_to_tf_paras(header_, data_, paras)
    msg = tfs.transforms[0]

    assert msg.header.frame_id == base_id
    assert msg.child_frame_id == child_id


def test_data_to_imu():
    raw_data = '%CORRIMUSA,2269,129755.030;1,-0.0000731120567875,'\
        '-0.0000083534662499,0.0000521803559345,-0.0007077320118335,'\
        '0.0008881368174285,0.0001594393292871,7.090,0*b388322e'.split(';')
    header_ = raw_data[0].split(',')
    data_ = raw_data[1].split(',')
    IMU_DATA_RATE = 100
    frame_id = 'vehicle'
    msg = data_to_imu(header_, data_, (IMU_DATA_RATE, frame_id))

    assert msg.header.frame_id == frame_id
    assert msg.angular_velocity.x == pytest.approx(-0.00731120567875)
    assert msg.angular_velocity.y == pytest.approx(-0.00083534662499)
    assert msg.angular_velocity.z == pytest.approx(0.00521803559345)
    assert msg.linear_acceleration.x == pytest.approx(-0.07077320118335)
    assert msg.linear_acceleration.y == pytest.approx(0.08881368174285)
    assert msg.linear_acceleration.z == pytest.approx(0.01594393292871)
