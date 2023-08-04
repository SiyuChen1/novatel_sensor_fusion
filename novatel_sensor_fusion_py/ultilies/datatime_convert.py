from datetime import datetime, timedelta


def utc_to_gps_time(utc_time):
    # GPS start time
    gps_start_time = datetime(1980, 1, 6, 0, 0, 0)

    # Number of leap seconds as of the knowledge cut-off date
    leap_seconds = 18

    # Calculate the difference between UTC time and GPS start time
    delta_time = utc_time - gps_start_time

    # Add the leap seconds to get GPS time
    # GPS time = UTC + 18s at present
    gps_time_seconds = delta_time.total_seconds() + leap_seconds

    # Convert to a datetime object
    gps_time = gps_start_time + timedelta(seconds=gps_time_seconds)

    return gps_time


def utc_to_gps_week_seconds(utc_time):
    gps_start_time = datetime(1980, 1, 6)
    leap_seconds = 18  # Update this value as needed
    delta_time = utc_time - gps_start_time + timedelta(seconds=leap_seconds)
    gps_week = delta_time.days // 7
    gps_seconds = delta_time.seconds + 86400 * (delta_time.days % 7) + delta_time.microseconds / 1e6
    return gps_week, gps_seconds
