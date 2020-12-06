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
    def __init__(self, cmdName, x, y, z, r, u, v):
        self.cmdName = cmdName
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.u = u
        self.v = v
        self.pos = np.array((x, y, z))
        self.pos2d = np.array((u, v))

# TODO: Modify these values based on our final specs. At this point, not being used in favor of a param in CVSpheres.
# KNOWN_WIDTH = 3.4925 # cm
# KNOWN_DISTANCE = 5.08 # cm

class CVSpheres:
    def __init__(self, hardwareCameraId, knownWidth, knownDistance, *commandColors):
        self.knownWidth = knownWidth
        self.knownDistance = knownDistance

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
    def __zDistanceToCamera(self, perceivedWidth):
        return (self.knownWidth * self.focalLength) / perceivedWidth

    # Calibration should only need to run once, but choose a color with which to do it so we know what to look for.
    # User should be able to trigger when the photo is taken, or perhaps just use a photo already taken in (specify image name).
    def calibrate(self, cmdName, img):
        # Find sphere in photo
        circles = self.findCircles(img)[cmdName]
        if circles is None or len(circles) == 0:
            print(f"Failed to calibrate {cmdName}: could not find any circles of that type.")
            return False # Failed
        
        # Get width (or bbox, or center and radius) of sphere
        (x, y, r) = circles[0]
        width = r * 2.0
        self.focalLength = (width * self.knownDistance) / self.knownWidth
        self.pixelToRealRatio = self.knownDistance / self.knownWidth
        return True # Succeeded

    def findCircles(self, img, showImgs=False, live=False):
        # ret, frame = self.cap.read()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        masks = dict()
        circles = dict()

        # FIXME: Going forward, we might want to generate parameters based instead on the size of the largest blob.
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

    # TODO: Note that this is untested! Will test soon, but wanted to push this for now.
    def findSpheres(self, circles):
        # Convert circles to spheres
        spheres = []
        for cmdName in circles:
            for x, y, r in circles[cmdName]:
                # Get depth
                depth = self.__zDistanceToCamera()
                # Convert x, y to camera coords
                X = depth * x
                Y = depth * y
                Z = depth
                # Convert r to real width, if that's even necessary
                R = (2.0 * r) / self.pixelToRealRatio
                spheres.append(Sphere(cmdName, X, Y, Z, R, x, y))
        return spheres

    def toEndEffectorCoords(self, x, y, z):
        # TODO: Depends on final length of robot arm and position of camera relative to end effector. Should be a simple translation.
        pass

    def toSpatialCoords(self, g_st, x, y, z):
        pEndEffector = self.toEndEffectorCoords(x, y, z)
        # Go back from tool frame to spatial frame
        return np.matmul(g_st, pEndEffector)

# You'll need to choose your own colors based on lighting conditions and the ping pong ball colors we get.
# I highly recommend having two different thresholds per ping pong ball: one for daytime, one for nighttime.
# Then, just keep lighting conditions consistent between those two environments, and you're set!
if __name__ == '__main__':
    ccOrange = CommandColor(
        'testOrange',
        (4, 113, 136),
        (8, 193, 245)
    )
    ccPureBlue = CommandColor(
        'testPureBlue',
        (112, 222, 212),
        (114, 246, 246)
    )
    ccBlue = CommandColor(
        'testBlue',
        (101, 60, 41),
        (110, 255, 255)
    )
    ccLime = CommandColor(
        'testLime',
        (56, 146, 40),
        (82, 255, 255)
    )
    
    cvs = CVSpheres(0, ccBlue, ccLime)

    while True:
        img = cvs.takePhoto(ui=False)
        cvs.findCircles(img, showImgs=True, live=True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

