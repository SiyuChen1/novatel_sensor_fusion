from datetime import timezone, timedelta, datetime


# Assuming Berlin's time zone as UTC+1 for this demonstration (not accounting for DST automatically)
berlin_offset = timedelta(hours=1)


# Function to convert POSIX timestamp to GPS week and seconds, considering Berlin's time zone
def posix_to_gps_berlin(posix_timestamp, timezone_offset):
    # Define GPS epoch start date
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

    # Convert POSIX timestamp to datetime in UTC
    posix_time_utc = datetime.utcfromtimestamp(posix_timestamp)

    # Adjust for Berlin's timezone (manually)
    posix_time_berlin = posix_time_utc.replace(tzinfo=timezone.utc) + timezone_offset

    # Calculate total seconds since GPS epoch
    total_seconds_since_gps_epoch = (posix_time_berlin - gps_epoch).total_seconds()

    # Calculate GPS week and seconds
    gps_week = int(total_seconds_since_gps_epoch // (7 * 24 * 3600))
    gps_seconds = total_seconds_since_gps_epoch % (7 * 24 * 3600)

    return gps_week, gps_seconds


# POSIX timestamps to be converted to GPS week and time
posix_timestamps = [
    (1707822196.8, 1707822286.7),  # First range
    (1707823522.2, 1707823622.1)  # Second range
]

# Convert given POSIX timestamps to GPS week and time
gps_conversions = [(posix_to_gps_berlin(start, berlin_offset), posix_to_gps_berlin(end, berlin_offset)) for start, end in posix_timestamps]

print(gps_conversions)