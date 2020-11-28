# Input:  Images from some webcam.
# Output: Positions of as many command spheres (and what colors) are in the environment, to then be interpreted.
# Other things that need to happen in between:
# - Callibration: How far away are the spheres? What's our initial measure of their distance?
# - Segmentation: How saturated or desaturated must a sphere be to stand out? Should we be using ArUco tags instead?

# Edge cases that come to mind:
# - Two spheres on "top" of each other (overlapping in image coords).
#   - Weirdest if both are same color. Probably less weird if they're different colors; algo can tell them apart.

import cv2
import numpy as np
import time

# Define commands and their color thresholds
class CommandColor:
    def __init__(self, name, minColor, maxColor):
        self.name = name
        self.minColor = minColor
        self.maxColor = maxColor

class Sphere:
    def __init__(self, cmdName, x, y, z):
        self.cmdName = cmdName
        self.x = x
        self.y = y
        self.z = z
        self.pos = np.array((x, y, z))

# TODO: Possibly modify these values based on our final specs.
KNOWN_WIDTH = 3.4925 # cm
KNOWN_DISTANCE = 5.08 # cm

# TODO: Look into perspective projection matrix
# Depth and x and y, so we'd know how far out it is. So basically multiply x and y by the width, or some multiple of the width, to get the depth. 
# lambda * homogeneous coordinates = spatial frame yay (maybe calibrate camera matrix first)? Then transform in FK.

# For sphere identification: check max width and min width, and if they're roughly equal, 
# Use kmeans from scipy for clustering/throwing out outliers. kmeans will give you clusters.
# Try blob detectors from OpenCV.

# Write a function that tells us the dela between the current position and the desired position
# Want the coords in base frame, so write a helper function that takes end effector transform and then translates backward a bit.

class CVSpheres:
    def __init__(self, hardwareCameraId, *commandColors):
        self.cap = cv2.VideoCapture(hardwareCameraId)
        self.focalLength = -1.0 # Should be initialized by self.calibrate

        self.commandColors = dict()
        for cmd in commandColors:
            self.commandColors[cmd.name] = cmd

    def takePhoto(self, ui=True):
        if not ui:
            ret, frame = self.cap.read()
            return frame
        
        frame = None
        while True:
            ret, frame = self.cap.read()
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        return frame

    # Source (also for calibration): https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
    def __zDistanceToCamera(self, knownWidth, focalLength, perceivedWidth):
        return (knownWidth * focalLength) / perceivedWidth

    # Calibration should only need to run once, but choose a color with which to do it so we know what to look for.
    # User should be able to trigger when the photo is taken, or perhaps just use a photo already taken in (specify image name).
    def calibrate(self, cmdName):
        # TODO: Take photo
        
        # TODO: Find sphere in photo
        # TODO: Get width (or bbox, or center and radius) of sphere
        self.focalLength = (width * KNOWN_DISTANCE) / KNOWN_WIDTH

    def findCircles(self, img, showImgs=False, live=False):
        # ret, frame = self.cap.read()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        masks = dict()
        circles = dict()
        blur_amt = 18
        houghMinDistance = 20

        maskMainImgs = []
        imgOutputs = []
        maskImgs = []

        for name, cmd in self.commandColors.items():
            masks[name] = cv2.inRange(hsv_img, cmd.minColor, cmd.maxColor)
            maskMainImgs.append(masks[name])

            maskImg = masks[name]
            maskImg = cv2.morphologyEx(maskImg, cv2.MORPH_CLOSE, np.ones((20,20),np.uint8))
            maskImg = cv2.morphologyEx(maskImg, cv2.MORPH_OPEN, np.ones((10,10),np.uint8))
            maskImg = cv2.dilate(maskImg, np.ones((10,10),np.uint8), iterations=1)
            maskImg = cv2.blur(maskImg, (blur_amt,blur_amt), 0)
            maskImg = cv2.erode(maskImg, np.ones((10,10),np.uint8), iterations=1)
            maskImgs.append(maskImg)
            
            if showImgs and not live:
                cv2.imshow("blurred", np.hstack([masks[name], maskImg]))
                cv2.waitKey(0)

            # FIXME: 100 should be based instead on the size of the largest blob.
            myCircles = cv2.HoughCircles(maskImg, cv2.HOUGH_GRADIENT, 1.2, houghMinDistance) # TODO: Tweak these parameters further
            if myCircles is None:
                print(f"No circles found for command {name}")
            
            if showImgs:
                myCircles2 = [] if myCircles is None else np.round(myCircles[0, :]).astype('int')
                output = cv2.bitwise_and(img, img, mask=masks[name])
                for (x, y, r) in myCircles2:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x-5, y-5), (x+5, y+5), (0,128,255), -1)
                if not live:
                    cv2.imshow("output", np.hstack([img, output]))
                    cv2.waitKey(0)
                imgOutputs.append(output)

            circles[name] = myCircles

        if showImgs and live:
            cv2.imshow("output-live", np.vstack([
                np.hstack([img, sum(imgOutputs)]),
                np.hstack([cv2.cvtColor(sum(maskMainImgs), cv2.COLOR_GRAY2BGR), cv2.cvtColor(sum(maskImgs), cv2.COLOR_GRAY2BGR)])
            ]))

        return circles

if __name__ == '__main__':
    ccOrange = CommandColor(
            'testOrange',
            # (0.0470 * 180.0, 0.4490 * 255, 0.9608 * 255),
            # (0.0259 * 180.0, 0.7518 * 255, 0.5373 * 255)
            (4, 113, 136),
            (8, 193, 245)
            # (2, 100, 120),
            # (10, 200, 255)
        )
    ccPureBlue = CommandColor(
        'testPureBlue',
        # (0.6296 * 180.0, 0.9551 * 255, 0.9608 * 255),
        # (0.6324 * 180.0, 0.8726 * 255, 0.8314 * 255)
        (112, 222, 212),
        (114, 246, 246)
    )
    ccBlue = CommandColor(
        'testBlue',
        # (104.994, 60.384, 227.9955),
        # (102.582, 192.8055, 41.004)
        # (101, 60, 41),
        # (105, 245, 228)
        # (101, 60, 41),
        # (105, 245, 228)
        (101, 60, 41),
        (110, 255, 255)
    )
    ccLime = CommandColor(
        'testLime',
        # (56.898, 146.931, 151.011),
        # (74.484, 255, 233.988),
        # (74.592, 255, 255),
        # (81.414, 255, 255),
        # (56, 146, 151),
        # (82, 255, 255)
        (56, 146, 40),
        (82, 255, 255)
    )
    
    cvs = CVSpheres(0, ccBlue, ccLime)
    # img = cv2.imread('/Users/jessiemindel/Downloads/blue-sphere-calibrate-b2.jpg',1)
    # img = cv2.imread('/Users/jessiemindel/Downloads/two-colors.jpg',1)

    while True:
        img = cvs.takePhoto(ui=False)
        cvs.findCircles(img, showImgs=True, live=True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

