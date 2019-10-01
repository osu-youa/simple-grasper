#!/usr/bin/env python

# Used to plot the results found

import sys
import os
import cPickle
import matplotlib.pyplot as plt
import LMD_plots as lmd_plt
import csv
import numpy as np

write_csv = False                                   # set true for if you want to copy out to a CSV file
save_plot = True                                    # set true if you want to save these plots
file_path = '/catkin_ws/src/simple-grasper/Data/'         # location to save the plots, or CSV files

def write2csv(data, header, name):
    """ This writes the selected data to a csv file to be probably checked out in excel. Fancy fancy!
    :param data: the data to be written [t, data1, data2, datan]
    :param header: the labels to be displayed at the top of the csv file labeling each data column [time, balsd..]
    """
    title = path + name + ".csv"
    with open(title, 'w') as csvFile:
        writer = csv.writer(csvFile, delimiter=',', lineterminator='\n')
        writer.writerow(header)                 # first write the header to the data
        writer.writerows(data)                  # then add the actual desired data

    csvFile.close()
    print("done writing CSV to file {}".format(title))


if len(sys.argv) == 1:
    raise ValueError('Type the name of a bag file to inspect')

bag_file = sys.argv[1]
root = os.path.expanduser('~')
path = os.path.join(root, file_path, bag_file)
# path = os.path.join(root, bag_file)
file_title = root+file_path+bag_file
with open(file_title, 'rb') as fh:
    results = cPickle.load(fh)

# Example of how to analyze - your analysis will be different!
times = []
force_x = []
IMU_accel = []
IMU_gyro = []
IMU_mag = []
IMU_rpy = [[0, 0, 0, 0]]

for wrench in results['wrench']:
    times.append(wrench.header.stamp.to_sec())
    force_x.append(wrench.wrench.force.x)

plt.plot(times, force_x)


###############################
# LMD Addition
###############################

for rpy in results['rpy_data']:
    time = rpy.header.stamp.to_sec()
    values = [rpy.vector.x, rpy.vector.y, rpy.vector.z]
    IMU_rpy = np.append(IMU_rpy, [[time, rpy.vector.x, rpy.vector.y, rpy.vector.z]], axis=0)

if write_csv:
    # use the whole data set of one 2d list, then only use the second column to not get duplicate time
    data_for_csv = np.stack((IMU_rpy[:, 0], IMU_rpy[:, 1], IMU_rpy[:, 1], IMU_rpy[:, 1]), axis=1)
    labels = ["Time [s]", "roll [deg]", "pitch [deg]", "yaw [deg]"]
    file_title = "RPY"
    write2csv(data_for_csv, labels, file_title)


time = IMU_rpy[:, 0]
plot_a = IMU_rpy[:, 1]
plot_b = IMU_rpy[:, 2]
plot_c = IMU_rpy[:, 3]
x_label = 'Time [s]'
y_labels = ["Roll [deg]", "Pitch [dag]", "Yaw [deg]"]
title = "RPY"
lmd_plt.lmd_three_subplot(time, plot_a, plot_b, plot_c, x_label, y_labels, title, save_plot, root+file_path)

plt.show()
