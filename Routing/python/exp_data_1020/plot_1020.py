# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.interpolate import splprep, splev
import os

plt.rcParams['font.sans-serif'] = ['Droid Sans Fallback']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负


def get_spline(path_x, path_y, heading):
    headingVecLength = 0.5
    headingVec = [headingVecLength * math.cos(math.pi - heading), headingVecLength * math.sin(math.pi - heading)]
    pts = []
    for i, (x, y) in enumerate(zip(path_x, path_y)):
        if i == 1:
            pts.append((path_x[0] + headingVec[0], path_y[0] + headingVec[1]))
        pts.append([x, y])
    pts = np.array(pts)
    tck, u = splprep(pts.T, u=None, s=0.2, per=0)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)
    return x_new, y_new


quiver_settings_1 = {'scale': 1,
                     'units': 'xy',
                     'width': 0.2}

quiver_settings_2 = {'scale': 1,
                     'units': 'xy',
                     'width': 0.15,
                     'color': 'g'}


def map_path_plot(wind, heading, path_x, path_y, obs_x, obs_y, map_size):
    n_row, n_col = map_size
    print('wind: {}'.format(wind))
    print('heading: {}'.format(heading))
    if len(path_x) > 1:
        path_x_f, path_y_f = get_spline(path_x, path_y, heading)

    windVecLength = 1.8
    windVec = [windVecLength * math.cos(wind), windVecLength * math.sin(wind)]

    headingVecLength = 1.8
    headingVec = [headingVecLength * math.cos(heading), headingVecLength * math.sin(heading)]

    plt.figure('map_path')
    # plt.figure(figsize=(6, 4), dpi=98)
    p1 = plt.subplot(111)
    if len(path_x) > 1:
        p1.plot(np.array(path_y), np.array(path_x), label=u'规划路径')
        p1.plot(np.array(path_y_f), np.array(path_x_f), label=u'光滑路径')
        p1.scatter([path_y[0]], [path_x[0]], s=20, c='green')  # start
        p1.scatter([path_y[-1]], [path_x[-1]], s=20, c='green')  # end
        p1.text(path_y[0], path_x[0], s=u'起点', fontsize=8, color='black')
        p1.text(path_y[-1], path_x[-1], s=u'终点', fontsize=8, color='black')

        p1.quiver(path_y[0], path_x[0], np.ones((1, 1)) * headingVec[1], np.ones((1, 1)) * headingVec[0],
                  **quiver_settings_2)
    p1.plot(np.array(obs_y), np.array(obs_x), 'ro', label=u'障碍物')

    p1.quiver(np.array(2), np.array(2), np.ones((1, 1)) * windVec[1], np.ones((1, 1)) * windVec[0], **quiver_settings_1)
    p1.text(2, 2, s=u'风向', fontsize=8, color='magenta')

    # p1.set_title('upwind')
    p1.legend(loc='upper right', fontsize=8)
    p1.axis([0, n_col - 1, n_row - 1, 0])
    p1.set_xticks(np.arange(0, n_col, 1))
    p1.tick_params(labelsize=6)
    p1.set_yticks(np.arange(0, n_row, 1))
    p1.xaxis.tick_top()
    p1.grid(linestyle='--', linewidth=0.5)
    p1.set_aspect(1)

    plt.tight_layout()

    file_folder = 'test_imgs/'
    if not os.path.exists(file_folder):
        os.system('mkdir -p ' + file_folder)
    # file_path = os.path.join(file_folder, '{}.eps'.format("test"))
    file_path = os.path.join(file_folder, '{}.svg'.format("test"))
    # file_path = os.path.join(file_folder, '{}.png'.format("test"))
    print("saving file {} ...".format(file_path))
    plt.savefig(file_path)
    plt.show()

    plt.close('all')


def wind_plot(wind_list):
    wind_array = np.unwrap(np.array(wind_list))
    # wind_array = np.array(wind_list)
    plt.figure('wind')
    plt.plot(wind_array * 57.3)
    plt.show()

def path_ne_plot(ego_x_list, ego_y_list, path_ne_x, path_ne_y, n):
    plt.figure('path_ne')
    p1 = plt.subplot(111)

    p1.plot(np.array(ego_y_list[0:n]), np.array(ego_x_list[0:n]))
    p1.plot(np.array(ego_y_list[n:-1]), np.array(ego_x_list[n:-1]), '--')
    p1.plot(np.array(path_ne_y), np.array(path_ne_x))

    p1.set_aspect(1)

    plt.show()



if __name__ == '__main__':

    map_size_list = []
    obs_row_list, obs_col_list = [], []
    with open('map_1020.txt', 'r') as f:
        obs_x, obs_y = [], []
        row = 0
        for line in f.readlines():
            if line.startswith('####'):
                obs_row_list.append(obs_x)
                obs_col_list.append(obs_y)
                obs_x, obs_y = [], []
                n_row, n_col = row, col + 1
                map_size_list.append((n_row, n_col))
                row = 0
                continue
            line = line.rstrip('\n').rstrip(' ').split(' ')
            for col, num in enumerate(line):
                if int(num) == 1:
                    obs_x.append(row)
                    obs_y.append(col)
            row += 1
    print(map_size_list[-1])
    print(len(map_size_list))
    print(obs_row_list[-1])
    print(obs_col_list[-1])

    heading_list = []
    wind_list = []
    path_row_list, path_col_list = [], []
    path_x_list, path_y_list = [], []
    obs_x_list, obs_y_list = [], []
    ego_x_list, ego_y_list = [], []
    with open('path_1020.txt', 'r') as f:
        path_row, path_col = [], []
        path_x, path_y = [], []
        for line in f.readlines():
            if line.startswith('ego_pos'):
                tmp = line.rstrip('\n').split(',')
                ego_x_list.append(float(tmp[1]))
                ego_y_list.append(float(tmp[2]))
                continue
            if line.startswith('obs_pos'):
                tmp = line.rstrip('\n').split(',')
                obs_x_list.append(float(tmp[1]))
                obs_y_list.append(float(tmp[2]))
                continue
            if line.startswith('heading'):
                heading_tmp = line.rstrip('\n').split(',')
                heading = float(heading_tmp[1])
                heading_list.append(heading)
                continue
            elif line.startswith('wind'):
                wind_tmp = line.rstrip('\n').split(',')
                wind_list.append(float(wind_tmp[1]))
                continue
            elif line.startswith('####'):
                path_row_list.append(path_row)
                path_col_list.append(path_col)
                path_row, path_col = [], []
                path_x_list.append(path_x)
                path_y_list.append(path_y)
                path_x, path_y = [], []
                continue
            line = line.rstrip('\n').split(',')
            path_row.append(float(line[0]))
            path_col.append(float(line[1]))
            path_x.append(float(line[2]))
            path_y.append(float(line[3]))

    print('wind_list[-1]: {}'.format(wind_list[-1]))
    print('heading_list[0]: {}'.format(heading_list[0]))
    print(len(path_x_list))
    print(path_x_list[-1])

    n = 700
    # map_path_plot(wind_list[n], heading_list[n], path_row_list[n], path_col_list[n], obs_row_list[n], obs_col_list[n],
    #               map_size_list[n])
    #
    # wind_plot(wind_list)

    path_ne_plot(ego_x_list, ego_y_list, path_x_list[n], path_y_list[n], n)