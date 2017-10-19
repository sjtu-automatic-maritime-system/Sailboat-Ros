import numpy as np
import utils

# BOAT_LENGTH = 1.5
DISTANCE_CAM_TO_WATER = 0.3

cameraMatrix = np.matrix([816.118268598647, 0.000000, 680.6511245884145,
                          0.000000, 822.0196620588329, 458.230641061779,
                          0, 0, 1]).reshape(3, 3)
FOCAL_LENGTH = 820.0
IMG_CENTER_Y = 458.0
IMG_CENTER_X = 681.0

K = np.concatenate((cameraMatrix, np.array([0, 0, 0]).reshape(1, -1)), axis=0)
K = np.concatenate((K, np.array([0, 0, 0, 0]).reshape(-1, 1)), axis=1)

R_boat_to_cam = utils.get_rotation_matrix(0, -np.pi / 2, -np.pi / 2)
R_boat_to_cam = np.matrix(R_boat_to_cam)

t_boat_to_cam = np.array([0, DISTANCE_CAM_TO_WATER, 0])
T_boat_to_cam = utils.transform_from_rot_trans(R_boat_to_cam, t_boat_to_cam)

P_boat_to_img = K*T_boat_to_cam
M_mat = P_boat_to_img[0:3, 0:3]
p_4 = P_boat_to_img[0:3, 3]
M_mat_inv = M_mat**-1
C_tidle = -M_mat_inv*p_4

def img_to_boat(u, v):
    h_pt_img = np.matrix([u, v, 1]).reshape(3, 1)
    X_tidle = M_mat_inv * h_pt_img
    mue_N = -C_tidle[2] / X_tidle[2]
    mue_N = mue_N[0, 0]
    # print(mue_N)
    pt_boat = mue_N * X_tidle + C_tidle
    # print(pt_boat)
    return np.squeeze(np.asarray(pt_boat))