# IMU Aided KLT
Python proof of concept using RealSense D435i camera. This module captures
image and IMU data from the RealSense camera, integrates the IMU data to
get position, warps the last retrieved image using that position, and 
then subtracts the warped image from the current image. The user can specify
how many image frames to skip in `params.py`. 

# Start
1. Connect to the RealSense camera
2. Open a terminal and run `python real_sense.py`