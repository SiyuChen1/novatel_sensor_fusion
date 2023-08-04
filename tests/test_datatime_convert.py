from datetime import datetime, timedelta
from gps_time import GPSTime
from novatel_sensor_fusion_py.ultilies.datatime_convert import utc_to_gps_time, utc_to_gps_week_seconds
import pytest


def test_utc_to_gps_time():
    # Define a UTC time
    utc_time = datetime(2021, 9, 1, 12, 0, 0)

    # Expected GPS time (you should replace this with the correct GPS time)
    expected_gps_time = datetime(2021, 9, 1, 12, 0, 18)

    # Call the utc_to_gps_time function
    gps_time = utc_to_gps_time(utc_time)

    # Check if the result matches the expected GPS time
    assert gps_time == expected_gps_time, f"Expected {expected_gps_time}, but got {gps_time}"


def test_utc_to_gps_week_seconds():
    # Define a UTC time
    utc_time = datetime(2021, 9, 1, 12, 0, 0, int(7.8e3))

    gps_time = GPSTime.from_datetime(utc_time + timedelta(seconds=18))

    # Expected GPS week and seconds into the week (you should replace these with the correct values)
    expected_gps_week = gps_time.week_number
    expected_gps_seconds = gps_time.time_of_week

    # Call the utc_to_gps_week_seconds function
    gps_week, gps_seconds = utc_to_gps_week_seconds(utc_time)

    # Check if the results match the expected GPS week and seconds
    assert gps_week == expected_gps_week, f"Expected GPS Week {expected_gps_week}, but got {gps_week}"
    assert gps_seconds == pytest.approx(expected_gps_seconds), f"Expected GPS Seconds {expected_gps_seconds}, but got {gps_seconds}"

