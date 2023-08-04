from datetime import datetime
from novatel_sensor_fusion_py.ultilies.datatime_convert import utc_to_gps_week_seconds
import pandas as pd
import re


# Function to read the IMU data into a Pandas DataFrame
def read_data_into_dataframe(file_path):
    # Reading the file line by line
    lines = []
    with open(file_path, 'r') as file:
        for line in file:
            # This line contains information
            if line.strip():
                lines.append(line)

    # Creating a DataFrame with the raw lines
    df = pd.DataFrame(lines, columns=['raw_line'])

    return df


# Function to parse the GPS time and other relevant fields from the raw line
def parse_line(row):
    line = row['raw_line']

    # Splitting the line by comma to identify the command type
    parts = line.split(",")
    if len(parts) < 2:
        return None

    # Extracting the command type
    # command_type = parts[0].split("#")[-1]
    command_type = re.split('[#%]', parts[0])[-1]

    # # Removing the "A" at the end of the command type as instructed
    # command_type = command_type[:-1] if command_type.endswith("A") else command_type

    # Check if the command type is one of the relevant ones (e.g., BESTPOS, BESTVEL)
    if command_type in ['INSATTXA', 'BESTPOSA', 'BESTVELA',
        'BESTGNSSPOSA', 'BESTGNSSVELA', 'BESTXYZA',
        'BESTXYZA', 'BESTUTMA']:
        # Extracting the GPS time (assuming the format based on the manual)
        # parts[5]: GPS week
        # parts[6]: Seconds after GPS week
        gps_time = parts[5] + ',' + '{:.3f}'.format(float(parts[6]))
    elif command_type in ['INSATTQSA', 'CORRIMUSA']:
        gps_time = parts[1] + ',' + '{:.3f}'.format(float(parts[2].split(';')[0]))
    elif command_type in ['GPGGA', 'GPRMC']:
        utc_time_str = parts[1].split('.')
        utc_time_str_first = utc_time_str[0]
        utc_time_microsecs = float('0.' + utc_time_str[1]) * 1e6
        utc_time = datetime(
            2023, 7, 3, int(utc_time_str_first[0:2]), int(utc_time_str_first[2:4]), int(utc_time_str_first[4:6]),
            int(utc_time_microsecs))
        gps_week, gps_seconds = utc_to_gps_week_seconds(utc_time)
        gps_time = str(gps_week) + ',' + '{:.3f}'.format(gps_seconds)
    else:
        gps_time = None

    if gps_time is not None:
        # Storing the relevant information
        return pd.Series({
            'command_type': command_type,
            'gps_time': gps_time,
            'raw_line': line
        })
    else:
        return None
