import numpy as np
import cv2
import os

class BallDetector:
    def __init__(self):
        pass

    def detect(self,image,name):

        cv2.imshow("name",image)
        cv2.waitKey(1)

        if type(image) == bytes:
            np_image = np.frombuffer(image, dtype=np.uint8)

            cv_image = cv2.imdecode(np_image,cv2.IMREAD_COLOR)
        else:
            cv_image = image

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # red_color = [0,255,255]
        # red_lower = np.array([0,20,20])
        # red_upper = np.array([10,255,255])
    
        # mask1 = cv2.inRange(hsv_image, red_lower, red_upper)

        # red_lower = np.array([170,20,20])
        # red_upper = np.array([180,255,255])

        # mask2 = cv2.inRange(hsv_image, red_lower, red_upper)

        # mask = cv2.bitwise_or(mask1,mask2)
    
        green_color = [60,255,255]
        green_lower = np.array([40,0,0])
        green_upper = np.array([65,255,255])
    
        # magenta_color = [100,255,255]
        # magenta_lower = np.array([95,0,0])
        # magenta_upper = np.array([105,255,255])

        mask = cv2.inRange(hsv_image, green_lower, green_upper)

        cv2.imshow("mask",mask)
        cv2.waitKey(1)

        # # kernel=np.ones((3,3),np.uint8)
        # # dilated=cv2.dilate(output,kernel,iterations=3)
        # gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        # apply binary thresholding
        img2 = mask.copy()
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        sizes = stats[1:, -1]; nb_components = nb_components - 1
        for i in range(0, nb_components):
            if sizes[i] <= 10:
                img2[output == i + 1] = 0
        kernel = np.ones((5,5), np.uint8)
        img2 = cv2.dilate(img2, kernel, iterations=1)

        # img2 = cv2.blur(img2, (3, 3),0)
        # ret, thresh = cv2.threshold(img2, 25, 255, cv2.TH
        # RESH_BINARY)
        # nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(thresh, connectivity=8)
        # visualize the binary image
        # cv2.imshow('Binary image', img2)
        # cv2.waitKey(1)

        # img2 = cv2.bitwise_not(img2)
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.blobColor = 255

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 256

        # # Filter by Area.
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 10000000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.8

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia =True
        params.minInertiaRatio = 0.5

        detector = cv2.SimpleBlobDetector_create(params)

        keypoints = detector.detect(img2)

        # small = cv2.resize(im_with_keypoints, (0,0), fx=0.5, fy=0.5)
        # cv2.imshow(name,small)
        # cv2.waitKey(1)

        image_copy = cv_image.copy()
        pixel_locations = {}

        # for point in keypoints:

        #         x = int(point.pt[0])
        #         y = int(point.pt[1])
        #         l = int(point.size/1.9)

        #         cv2.rectangle(image_copy, (x - l, y - l), (x + l, y + l), (0, 0, 255), 2)

        #         cx = point.pt[0]
        #         cy = point.pt[1]

        #         cx_int = int(cx)
        #         cy_int = int(cy)
        #         # msg = "({}, {})".format(cx_int,cy_int)
        #         # print(msg)
        #         # hsv_color = hsv_image[cy_int,cx_int,:].copy()
        #         # hsv_color[0] = int(hsv_color[0])*2
        #         # print("color: HSV " + str(hsv_color) + "--" + self.which_color(hsv_image[cy_int,cx_int,:],uppers,lowers,key_colors))
        #         # print()

        #         cv2.circle(image_copy,(cx_int,cy_int), 1, (0,255,0),2)

        # cv2.imshow("detection", image_copy)
        # cv2.waitKey(1)

        return keypoints[0].pt

        # # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE

        # contours, hierarchy = cv2.findContours(image=img2, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)

        # # draw contours on the original image
        # image_copy = cv_image.copy()
        # pixel_locations = {}
        # # print("pixel locations of lights from top left:")
        # print(len(contours))
        # for cnt in contours:

        #         rect = cv2.minAreaRect(cnt)
        #         box = cv2.boxPoints(rect)
        #         box = np.int0(box)
        #         cv2.drawContours(image_copy,[box],0,(0,0,255),3)

        #         M = cv2.moments(cnt)
        #         if M['m00'] == 0:
        #             continue
        #         cx = float(M['m10']/M['m00'])
        #         cy = float(M['m01']/M['m00'])

        #         cx_int = int(cx)
        #         cy_int = int(cy)
        #         # msg = "({}, {})".format(cx_int,cy_int)
        #         # print(msg)
        #         # print("color: HSV " + str(hsv_image[cy_int,cx_int,:]) + "--" + self.which_color(hsv_image[cy_int,cx_int,:],all_colors,uppers,lowers,key_colors))
        #         # print()
        #         color_name = self.which_color(hsv_image[cy_int,cx_int,:],all_colors,uppers,lowers,key_colors)
        #         pixel_locations[color_name] = [cx,cy]

        # cv2.imwrite(os.path.normpath("points_detected" + '.png'), cv_image)
        # cv2.imwrite(os.path.normpath("points_detected" + '.png'), image_copy)

        # small = cv2.resize(image_copy, (0,0), fx=.5, fy=.5)
        # cv2.namedWindow(name)        # Create a named window
        # cv2.moveWindow(name, 0,0)  # Move it to (40,30)
        # cv2.imshow(name, small)
        # cv2.waitKey(1)
        # print("got it")
        # return None
        # return pixel_locations