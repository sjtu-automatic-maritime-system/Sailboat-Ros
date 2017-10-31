# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.interpolate import splprep, splev
import os

# import sys

plt.rcParams['font.sans-serif'] = ['Droid Sans Fallback']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负

target_x_list = [-60, -50, -20]
target_y_list = [30, -20, 0]

obs_num = 9

basename = '{}obs'.format(obs_num)

# map_file = '0obs/map_1028_0obs.txt'
# path_file = '0obs/path_1028_0obs.txt'

map_file = os.path.join(basename, 'map_1028_{}obs.txt'.format(obs_num))
path_file = os.path.join(basename, 'path_1028_{}obs.txt'.format(obs_num))


def wrap_angle(rad):
    while rad > np.pi:
        rad -= np.pi * 2
    while rad < -np.pi:
        rad += np.pi * 2
    return rad


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

quiver_settings_3 = {'scale': 0.5,
                     'units': 'xy',
                     'width': 0.4}


def map_path_plot(wind, heading, path_x, path_y, obs_x, obs_y, map_size):
    n_row, n_col = map_size
    # print('wind: {}'.format(wind))
    # print('heading: {}'.format(heading))
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

    file_folder = os.path.join(basename, 'imgs')
    if not os.path.exists(file_folder):
        os.system('mkdir -p ' + file_folder)
    # file_path = os.path.join(file_folder, '{}.eps'.format("test"))
    file_path = os.path.join(file_folder, 'map_frame{}.svg'.format(n))
    # file_path = os.path.join(file_folder, '{}.png'.format("test"))
    print("saving file {} ...".format(file_path))
    plt.savefig(file_path)
    # plt.show()

    # plt.close('all')


def wind_plot(wind_1, wind_2=None, unwrap=False):
    if unwrap is True:
        wind_1_array = np.unwrap(np.array(wind_1), discont=np.pi * 1.8)
    else:
        wind_1_array = np.array(wind_1)
    plt.figure('wind')
    plt.plot(wind_1_array * 57.3)
    if wind_2 is not None:
        if unwrap is True:
            wind_2_array = np.unwrap(np.array(wind_2), discont=np.pi * 1.8)
        else:
            wind_2_array = np.array(wind_2)
        plt.plot(wind_2_array * 57.3)
    plt.show()


def ne_path_plot(ego_x_list, ego_y_list, path_ne_x, path_ne_y, n, wind, obs_x, obs_y):
    print("frame {}, wind: {}".format(n, wind))
    plt.figure('ne_path')
    p1 = plt.subplot(111)

    p1.plot(np.array(ego_y_list[0:n]), np.array(ego_x_list[0:n]), 'b', label=u'当前路径')
    p1.plot(np.array(ego_y_list[n:-1]), np.array(ego_x_list[n:-1]), 'b--', label=u'全部路径')
    p1.plot(np.array(path_ne_y), np.array(path_ne_x), 'r', label=u'规划路径')

    p1.scatter(np.array(obs_y), np.array(obs_x), s=120, c='red', label=u'障碍物')

    p1.scatter(target_y_list, target_x_list, s=80, c='cyan')  # target
    for i in range(3):
        p1.text(target_y_list[i] + 1, target_x_list[i] + 1, s=u'目标{}'.format(i + 1), fontsize=12, color='black')

    p1.set_xlabel(u'东向 /m')
    p1.set_ylabel(u'北向 /m')

    xlim = p1.get_xlim()
    ylim = p1.get_ylim()
    quiver_x = xlim[0] + 0.1 * (xlim[1] - xlim[0])
    quiver_y = ylim[1] - 0.1 * (ylim[1] - ylim[0])
    windVecLength = min((xlim[1] - xlim[0]), (ylim[1] - ylim[0])) * 0.025
    windVec = [windVecLength * math.cos(wind), windVecLength * math.sin(wind)]

    p1.quiver(np.array(quiver_x), np.array(quiver_y), np.ones((1, 1)) * windVec[1], np.ones((1, 1)) * windVec[0],
              **quiver_settings_3)
    p1.text(quiver_x + 1, quiver_y + 1, s=u'风向', fontsize=12, color='magenta')

    p1.set_aspect(1)
    # p1.grid(linestyle='--', linewidth=0.5)
    p1.legend(loc='upper right', fontsize=12)

    file_folder = os.path.join(basename, 'imgs')
    if not os.path.exists(file_folder):
        os.system('mkdir -p ' + file_folder)
    # file_path = os.path.join(file_folder, '{}.eps'.format("test"))
    file_path = os.path.join(file_folder, 'ne_frame{}.svg'.format(n))
    # file_path = os.path.join(file_folder, '{}.png'.format("test"))
    print("saving file {} ...".format(file_path))
    plt.savefig(file_path)

    # plt.show()

    plt.close('all')


if __name__ == '__main__':

    map_size_list = []
    obs_row_list, obs_col_list = [], []
    with open(map_file, 'r') as f:
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
    wind_ave_list = []
    path_row_list, path_col_list = [], []
    path_x_list, path_y_list = [], []
    obs_x_list, obs_y_list = [], []
    ego_x_list, ego_y_list = [], []
    with open(path_file, 'r') as f:
        path_row, path_col = [], []
        path_x, path_y = [], []
        obs_x, obs_y = [], []
        for line in f.readlines():
            if line.startswith('ego_pos'):
                tmp = line.rstrip('\n').split(',')
                ego_x_list.append(float(tmp[1]))
                ego_y_list.append(float(tmp[2]))
                continue
            if line.startswith('obs_pos'):
                tmp = line.rstrip('\n').split(',')
                for i in range((len(tmp) - 1) / 2):
                    obs_x.append(float(tmp[2 * (i + 1) - 1]))
                    obs_y.append(float(tmp[2 * (i + 1)]))
                obs_x_list.append(obs_x)
                obs_y_list.append(obs_y)
                obs_x, obs_y = [], []
                continue
            if line.startswith('heading'):
                heading_tmp = line.rstrip('\n').split(',')
                heading = float(heading_tmp[1])
                heading_list.append(heading)
                continue
            elif line.startswith('wind'):
                wind_tmp = line.rstrip('\n').split(',')
                wind_list.append(wrap_angle(float(wind_tmp[1])))
                wind_ave_list.append(float(wind_tmp[2]))
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
    print('obs_x: {}'.format(obs_x_list[100]))
    print('obs_y: {}'.format(obs_y_list[100]))

    # wind_plot(wind_list, wind_ave_list)
    wind_plot(wind_ave_list, unwrap=False)

    for path_frame in range(20, 30):
        # path_frame = 24
        n = path_frame * 20 - 1
        map_path_plot(wind_ave_list[n], heading_list[n], path_row_list[path_frame], path_col_list[path_frame],
                      obs_row_list[path_frame], obs_col_list[path_frame],
                      map_size_list[path_frame])

        ne_path_plot(ego_x_list, ego_y_list, path_x_list[path_frame], path_y_list[path_frame], n, wind_ave_list[n],
                     obs_x_list[n], obs_y_list[n])
