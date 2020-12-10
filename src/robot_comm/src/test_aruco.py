import numpy as np
from cv2 import aruco
import cv2

import threading
class CameraBufferCleanerThread(threading.Thread):
    def __init__(self, camera, name='camera-buffer-cleaner-thread'):
        self.camera = camera
        self.last_frame = None
        super(CameraBufferCleanerThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()

# Defined colors
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

cap = cv2.VideoCapture(0)
cap_cleaner = CameraBufferCleanerThread(cap)

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()

class ArucoDetector():
    def __init__(self):
        self.ref_id = None # Set this when the first id is read

    def get_pos(self):
        curr_sphere_pos = None
        frame = cap_cleaner.last_frame

        if frame is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                if self.ref_id is None:
                    self.ref_id = ids[0][0] # take the first one

                ref_locs = np.where(ids.flatten()==self.ref_id)[0]
                if len(ref_locs) > 0:
                    ref_idx = ref_locs[0]

                    c = corners[ref_idx][0] # Take one of the ids
                    x_loc, y_loc = c[:, 0].mean(), c[:, 1].mean()
                    cv2.circle(frame, (x_loc, y_loc), 1, GREEN, thickness=4, lineType=8, shift=0)
                    
                    curr_sphere_pos = np.array([
                        (1 if x_loc > frame.shape[1]/2 else -1), 
                        (1 if y_loc > frame.shape[0]/2 else -1), 
                        0
                    ])
        
            cv2.imshow('frame', frame)

        # Is None if any of the above fails
        return curr_sphere_pos

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    while True:
        curr_sphere_pos = aruco_detector.get_pos()
        print('curr sphere pos', curr_sphere_pos)
        cv2.waitKey(5)
