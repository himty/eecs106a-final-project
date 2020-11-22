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
KNOWN_WIDTH = 2 # cm
KNOWN_DISTANCE = 5 # cm

class CVSpheres:
    def __init__(self, hardwareCameraId, *commandColors):
        self.cap = cv2.VideoCapture(hardwareCameraId)
        self.focalLength = -1.0 # Should be initialized by self.calibrate

        self.commandColors = dict()
        for cmd in commandColors:
            self.commandColors[cmd.name] = cmd

    # Source (also for calibration): https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
    def __zDistanceToCamera(knownWidth, focalLength, perceivedWidth):
        return (knownWidth * focalLength) / perceivedWidth

    # Calibration should only need to run once, but choose a color with which to do it so we know what to look for.
    # User should be able to trigger when the photo is taken, or perhaps just use a photo already taken in (specify image name).
    def calibrate(cmdName):
        # TODO: Take photo
        # TODO: Find sphere in photo
        # TODO: Get width (or bbox, or center and radius) of sphere
        self.focalLength = (width * KNOWN_DISTANCE) / KNOWN_WIDTH

    def findSpheres(self):
        ret, frame = self.cap.read()
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        masks = dict()
        for name, cmd in self.commandColors.items():
            masks[name] = cv2.inRange(hsv_img, cmd.minColor, cmd.maxColor)
            # TODO: But what do we now want to do with these? We want to find the spheres and their positions. The question is how.

        # TODO: Return an array of Spheres.
        return None
