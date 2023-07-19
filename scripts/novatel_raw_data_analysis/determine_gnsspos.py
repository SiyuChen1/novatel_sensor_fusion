import matplotlib.pyplot as plt
import pymap3d as pm

lat0, lon0, h0 = 50.77737927959, 6.07872928972, 177.8415


def get_data(abs_path, file_name):
    e_l = []
    n_l = []
    u_l = []
    with open(abs_path + file_name) as data_file:
        content = data_file.read().split('\n')
        for index, line in enumerate(content):
            # if "BESTGNSSPOSA" in line:
            if 'BESTPOSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                lat = float(data[2])
                lon = float(data[3])
                h = float(data[4])
                e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
                e_l.append(e)
                n_l.append(n)
                u_l.append(u)
    return e_l, n_l, u_l


folder_path = '/home/siyuchen/Documents/Novatel_Stereocam_Daten_20230703/'
file_names = ['Rotation um Antenne1_2023-07-03_14-36-08.log',
              'Rotation um IMU_2023-07-03_14-36-47.log']
labels = ['um Antenna', 'um IMU']
for i, f in enumerate(file_names):
    e_l, n_l, _ = get_data(folder_path, f)
    plt.plot(e_l, n_l, '*', label=labels[i])

# after visualisation, it can be determined that the gnss measurements are positions
# of the first antenna
plt.legend()
plt.axis('equal')
plt.show()
