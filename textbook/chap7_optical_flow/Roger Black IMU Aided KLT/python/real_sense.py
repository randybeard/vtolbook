# BYU MAGICC Lab
# date: April 2020
# name: Roger Black
# description: The following
# 1. opens a RealSense camera pipeline
# 2. captures image frames, acceleration, and gyro data
# 3. passes the raw IMU data through a low pass filter
# 4. calls the integrate function to get position from the acceleration and gyro data
# 5. passes the position to the warp function which warps the last retrieved image frame
# 6. compares the current image with the warped image
# Specify the number of frames to skip in params.py

import pyrealsense2 as rs
import numpy as np
import cv2 as cv
from low_pass_filter import LowPassFilter
import params as P
from integrate import Integrate
import warp as warp

integrate = Integrate()
lpf = LowPassFilter()


def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    conf.enable_stream(rs.stream.color)

    p.start(conf)
    return p


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])


def get_accel_gyro_data(f):
    accel = accel_data(f[1].as_motion_frame().get_motion_data())
    gyro = gyro_data(f[2].as_motion_frame().get_motion_data())
    return [accel, gyro]


def get_color_image(f):
    # get the color frame
    color_frame = f.get_color_frame()

    # convert the color frame to a color image
    color_image = np.asanyarray(color_frame.get_data())

    # convert the color image to grayscale
    img = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)
    return img


def accel_and_gyro_routine(f):
    [accel, gyro] = get_accel_gyro_data(f)
    accel = lpf.get_accel_estimate(accel)  # low pass filter
    gyro = lpf.get_gyro_estimate(gyro)  # low pass filter

    xyz_pos = integrate.acceleration_to_position(accel)
    angular_pos = integrate.gyro_to_position(gyro)
    return [xyz_pos[0], xyz_pos[1], xyz_pos[2], angular_pos[0], angular_pos[1], angular_pos[2]]


# TODO figure out how to record data to a file to be able to replay (.bag file?)
p = initialize_camera()
prev_img = np.asanyarray([])
warped_image = np.asanyarray([])
img = np.asanyarray([])
count = 0
pos = []
try:
    # initialize the script by getting an image and storing it as the previous image
    f = p.wait_for_frames()
    prev_img = get_color_image(f)
    while True:
        # get the next frame
        f = p.wait_for_frames()

        # if the number of frames to skip has NOT been met, only get the acceleration and gyro data
        if count < P.skip:
            pos = accel_and_gyro_routine(f)
            count += 1
        else:  # if the number of frames to skip has been met, get both IMU data and the current color image
            pos = accel_and_gyro_routine(f)
            img = get_color_image(f)
            warped_image = warp.warp_image(prev_img, pos)

            # reset iterative parameters
            count = 0
            integrate.reset()
            prev_img = img

            # subtract the images
            subtracted_img = cv.subtract(img, warped_image)

            # resize images (for viewing on a small screen)
            scale = 0.5
            img = cv.resize(img, (0, 0), None, scale, scale)
            warped_image = cv.resize(warped_image, (0, 0), None, scale, scale)
            subtracted_img = cv.resize(subtracted_img, (0, 0), None, scale, scale)

            # display the original video stream
            cv.namedWindow('Original', cv.WINDOW_AUTOSIZE)
            cv.imshow('Original', img)
            cv.waitKey(1)

            # display the warped video stream
            cv.namedWindow('Warped', cv.WINDOW_AUTOSIZE)
            cv.imshow('Warped', warped_image)
            cv.waitKey(1)

            # display the subtracted images
            cv.namedWindow('Subtracted', cv.WINDOW_AUTOSIZE)
            cv.imshow('Subtracted', subtracted_img)
            cv.waitKey(1)

finally:
    p.stop()
