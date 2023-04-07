import cv2
import numpy as np
from tools.dirty_derivative import DirtyDerivative
from parameters.quadrotor_parameters import K_inv
# from common import draw_str

class TimeToCollision:
    def __init__(self) -> None:
        self.first_image = True
        self.p0 = None
        self.prev_grey = None
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.tracks = []

        # Create random colors
        self.color = np.random.randint(0, 255, (100, 3))


    def compute_time(self, image, dt, draw=False):

        """Compute time to collision for a set of detected features based on optical flow from previous to current frame
        
        Input:
            image (np.ndarray rgb image): current image frame
            dt (double): time since last frame
            draw (optional - bool): flag for optical flow visualization
             
        Output:
            tauX, tauY: median computed time to collision in the x and y direction """

        # Create a mask image for drawing purposes
        if draw:
            mask = np.zeros_like(image)
            grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        if self.first_image:
            self.p0 = cv2.goodFeaturesToTrack(grey, mask=None, **self.feature_params)
            self.first_image = False
            self.prev_grey = grey
            return
        
        if len(self.p0) < 30:
            # Get more features if too many went out of field
            self.p0 = cv2.goodFeaturesToTrack(grey, mask=None, **self.feature_params)


        # Calculate Optical Flow in both directions to filter our bad matches
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_grey, grey, self.p0, None, **self.lk_params)
        p0r, st, err = cv2.calcOpticalFlowPyrLK(grey, self.prev_grey, p1, None, **self.lk_params)
        d = abs(self.p0-p0r).reshape(-1, 2).max(-1)
        good = d < 1
        
        # Select good points
        good_new = p1[good].reshape((-1,2))
        good_old = self.p0[good].reshape((-1,2))
       

        # # Draw the points
        if draw:
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.astype(int).ravel()
                c, d = old.astype(int).ravel()
                
                mask = cv2.line(mask, (a, b), (c, d), self.color[i].tolist(), 2)
                image = cv2.circle(np.copy(image), (a, b), 5, self.color[i].tolist(), -1)
    
            # Display the demo
            img = cv2.add(image, mask)
            cv2.imshow("frame", img)
            k = cv2.waitKey(25) & 0xFF

        # convert to calibrated pixels
        ones = np.ones([good_new.shape[0], 1])
        eta_new = np.concatenate((good_new, ones),1)
        eta_old = np.concatenate((good_old, ones),1)

        eta_new = (K_inv @ eta_new.T).T
        eta_old = (K_inv @ eta_old.T).T

        # Get velocity of each pixel
        dp = (eta_new - eta_old) / dt
        dx = dp[:,0]
        dy = dp[:,1]

        # Get time to collision for each pixel in x and y
        tauX = eta_new[:,0]/dx
        tauY = eta_new[:,1]/dy

        # Update the previous frame and previous points
        self.prev_grey = grey.copy()
        self.p0 = good_new.reshape(-1, 1, 2)

        return np.median(tauX), np.median(tauY)
        
            

        
        

