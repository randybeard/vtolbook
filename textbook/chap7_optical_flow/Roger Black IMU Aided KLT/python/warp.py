# BYU MAGICC Lab
# date: April 2020
# name: Roger Black
# description: warps a given image based on the given position
# this script is based heavily on the following post
# https://stackoverflow.com/questions/7019407/translating-and-rotating-an-image-in-3d-using-opencv

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import math as math
import params as P


def warp_image(img, pos):
    rows, cols = img.shape

    # Projection 2D -> 3D matrix
    A1 = np.float32(np.array([[1, 0, -cols / 2],
                              [0, 1, -rows / 2],
                              [0, 0, 0],
                              [0, 0, 1]]))

    # Rotation matrix
    rot = np.float32(np.array(pos[3:6]))

    # Use the Rodrigues function
    M_rot, _ = cv.Rodrigues(rot)

    bottom = np.array([0, 0, 0])
    right = np.array([0, 0, 0, 1])
    R = np.column_stack((np.vstack((M_rot, bottom)), right))

    # Translation matrix on the Z axis
    dist = P.dist
    T = np.float32(np.array([
        [1, 0, 0, pos[0]],
        [0, 1, 0, pos[1]],
        [0, 0, 1, dist+pos[2]],
        [0, 0, 0, 1]
    ]))

    # Camera Intrinsic matrix 3D -> 2D
    f = P.f
    A2 = np.float32(np.array([
        [f, 0, cols / 2, 0],
        [0, f, rows / 2, 0],
        [0, 0, 1, 0]
    ]))

    M = A2 @ T @ R @ A1

    dst = cv.warpPerspective(img, M, (cols, rows))

    return dst
