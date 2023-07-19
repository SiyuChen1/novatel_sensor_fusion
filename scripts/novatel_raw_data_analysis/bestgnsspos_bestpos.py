import matplotlib.pyplot as plt
import numpy as np


def plot_bestgnsspos_bestpos(abs_path, file_name):
    bestgnsspos = []
    bestpos = []
    gnss_std = []
    with open(abs_path + file_name) as data_file:
        content = data_file.read().split('\n')
        for index, line in enumerate(content):
            if 'BESTPOSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                lat = float(data[2])
                lon = float(data[3])
                h = float(data[4])
                bestpos.append([lat, lon, h])
            if 'BESTGNSSPOSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                lat = float(data[2])
                lon = float(data[3])
                h = float(data[4])

                lat_std = float(data[7])
                lon_std = float(data[8])
                h_std = float(data[9])
                gnss_std.append([lat_std, lon_std, h_std])

                bestgnsspos.append([lat, lon, h])
    return np.array(bestgnsspos), np.array(bestpos), np.array(gnss_std)


folder_path = '/home/siyuchen/Documents/Novatel_Stereocam_Daten_20230703/20230703_140222/'
file_name = 'IMUData_20230703_140222.log'
bestgnsspos, bestpos, gnss_std = plot_bestgnsspos_bestpos(folder_path, file_name)
# print(bestgnsspos.shape)
print(gnss_std.shape)

fig, axs = plt.subplots(3, 1)
axs[0].set_title('Latitude')
axs[0].plot(bestgnsspos[:, 0], label='bestgnsspos')
axs[0].plot(bestpos[:, 0], label='bestpos')
axs[0].legend()
par_0 = axs[0].twinx()
par_0.plot(gnss_std[:, 0], 'g', label='latitude std')
par_0.legend()
axs[0].ticklabel_format(useOffset=False)

axs[1].set_title('Longitude')
axs[1].plot(bestgnsspos[:, 1], label='bestgnsspos')
axs[1].plot(bestpos[:, 1], label='bestpos')
par_1 = axs[1].twinx()
par_1.plot(gnss_std[:, 1], 'g', label='longitude std')
par_1.legend()
axs[1].ticklabel_format(useOffset=False)
axs[1].legend()

axs[2].set_title('Altitude')
axs[2].plot(bestgnsspos[:, 2], label='bestgnsspos')
axs[2].plot(bestpos[:, 2], label='bestpos')
par_2 = axs[2].twinx()
par_2.plot(gnss_std[:, 2], 'g', label='altitude std')
par_2.legend()
axs[2].ticklabel_format(useOffset=False)
axs[2].legend()

plt.show()
