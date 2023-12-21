from gps_time import GPSTime


folder_path = '/home/siyuchen/Documents/Novatel_Stereocam_Daten_20230703/20230703_140222/'
raw_file_name = 'IMUData_20230703_140222.log'

processed_file_name = 'processed_IMUData_20230703_140222.log'

with open(folder_path + raw_file_name) as read_data_file:
    content = read_data_file.read().split('\n')
    with open(folder_path + processed_file_name, 'w') as write_data_file:
        for index, line in enumerate(content):
            if 'BESTPOSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                lat = data[2]
                lon = data[3]
                h = data[4]

                header = msg[0].split(',')
                cur_time = GPSTime(week_number=int(header[5]), time_of_week=float(header[6]))
                output = str(cur_time.to_datetime()) + ',' + lat + ',' + lon + ',' + h + '\n'
                write_data_file.write(output)
