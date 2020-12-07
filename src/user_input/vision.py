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
import json

from imgUtils import readImgsFromFolder

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

    def toString(self):
        return f"{self.cmdName} of radius {self.r} at spatial ({self.x}, {self.y}, {self.z}) and at camera ({self.u}, {self.v})"

class CVSpheres:
    def __init__(self, hardwareCameraId, knownWidth, knownDistance, *commandColors):
        self.knownWidth = knownWidth
        self.knownDistance = knownDistance

        self.cap = cv2.VideoCapture(hardwareCameraId)
        self.focalLength = -1.0 # Should be initialized by self.calibrate
        self.K = None # Should be configured by self.calibrateCameraMatrix
        self.invK = None # Should be configured by self.calibrateCameraMatrix

        self.commandColors = dict()
        for cmd in commandColors:
            self.commandColors[cmd.name] = cmd

    @staticmethod
    def fromConfigFile(path):
        cvs = None
        with open(path) as json_file:
            data = json.load(json_file)
            hardwareCameraId = data['hardwareCameraId']
            knownWidth = data['knownWidth']
            knownDistance = data['knownDistance']
            commandColors = []
            for c in data['commandColors']:
                cmd = CommandColor(c['name'], c['minColor'], c['maxColor'])
                commandColors.append(cmd)
            chessboardImgsPath = data['chessboardImgsPath']
            sphereCalibrationImgPath = data['sphereCalibrationImgPath']
            sphereCalibrationCmdName = data['sphereCalibrationCmdName']
            cvs = CVSpheres(hardwareCameraId, knownWidth, knownDistance, *commandColors)
            cvs.calibrateCameraMatrix(chessboardImgsPath)
            cvs.calibrate(sphereCalibrationCmdName, cv2.imread(sphereCalibrationImgPath, 1))
        return cvs

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

    # Source: https://nikatsanka.github.io/camera-calibration-using-opencv-and-python.html (+ OpenCV Python docs)
    # Also helpful: https://www.learnopencv.com/camera-calibration-using-opencv/
    # And another: https://stackoverflow.com/questions/16101747/how-can-i-get-the-camera-projection-matrix-out-of-calibratecamera-return-value
    def calibrateCameraMatrix(self, chessboardImgsPath):
        # Termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((7*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        imgs, filenames = readImgsFromFolder(chessboardImgsPath)
        
        found = 0
        for img in imgs:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (7,7), corners2, ret)
                found += 1
                cv2.imshow('img', img)
                cv2.waitKey(500)
                # if you want to save images with detected corners 
                # uncomment following 2 lines and lines 5, 18 and 19
                # image_name = path + '/calibresult' + str(found) + '.png'
                # cv2.imwrite(image_name, img)
        
        cv2.destroyAllWindows()

        # Calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        # Currently, I'm throwing away rvecs and tvecs, which would form the extrinsic camera matrix
        try:
            self.invK = np.linalg.inv(mtx)
            self.K = mtx
            return True
        except np.linalg.LinAlgError:
            return False

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

    def findCircles(self, img, largestOnly=False, showImgs=False, live=False):
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
            myCircles = myCircles if myCircles is None else myCircles[0]
            if myCircles is None:
                print(f"No circles found for command {name}")

            if largestOnly and myCircles is not None:
                # Select the circle with the largest radius
                myCircles = [max(myCircles, key=lambda x: x[2])]
            
            if showImgs:
                myCircles2 = [] if myCircles is None else np.round(myCircles).astype('int')
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

    def findSpheres(self, circles):
        if circles is None:
            return []
        # Convert circles to spheres
        spheres = []
        for cmdName in circles:
            if circles[cmdName] is None or len(circles[cmdName]) == 0:
                continue
            for circle in circles[cmdName]:
                u, v, r = circle
                # Get depth
                depth = self.__zDistanceToCamera(2.0 * r)
                # Convert u, v to camera coords and apply intrinsic camera matrix
                x = np.array([u,v,1])
                X = depth * np.matmul(self.invK, x) # FIXME: It's possible that invK won't exist at this point.
                # X = depth * x
                # Y = depth * y
                # Z = depth
                # Convert r to real width, if that's even necessary
                R = (2.0 * r) / self.pixelToRealRatio
                spheres.append(Sphere(cmdName, X[0], X[1], X[2], R, u, v))
        return spheres

    def toEndEffectorCoords(self, x, y, z):
        # TODO: Depends on final length of robot arm and position of camera relative to end effector. Should be a simple translation.
        return np.array([x,y,z])

    def toSpatialCoords(self, g_st, x, y, z):
        pEndEffector = self.toEndEffectorCoords(x, y, z)
        # Go back from tool frame to spatial frame
        return np.matmul(g_st, pEndEffector)

    def step(self):
        img = self.takePhoto(ui=False)
        circles = self.findCircles(img, largestOnly=True, showImgs=True, live=True)
        spheres = self.findSpheres(circles)
        transformedSpheres = []
        for s in spheres:
            X = self.toEndEffectorCoords(s.x, s.y, s.z)
            transformedSpheres.append(Sphere(s.cmdName, X[0], X[1], X[2], s.r, s.u, s.v))
        return transformedSpheres

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

    ccBlueShinyDay = CommandColor(
        'testBlue',
        (94, 31, 101),
        (108, 255, 252)
    )
    ccLimeMatteDay = CommandColor(
        'testLime',
        (27, 64, 101),
        (55, 218, 212)
    )
    
    KNOWN_WIDTH = 3.4925 # cm
    KNOWN_DISTANCE = 5.08 # cm
    cvs = CVSpheres(0, KNOWN_WIDTH, KNOWN_DISTANCE, ccBlue) # Currently only supports all objects being the same size.
    cvs.calibrateCameraMatrix('/Users/jessiemindel/Downloads/chessboard_calibrate')
    cvs.calibrate('testBlue', cv2.imread('/Users/jessiemindel/Downloads/blue-sphere-calibrate-b2.jpg', 1))

    # Uncomment and supply your own calibration image to test.
    # Results: works for z coordinate, and x and y coordinates look reasonable! Radius is... questionable.
    # testSpheres = cvs.findSpheres(cvs.findCircles(cv2.imread('/Users/jessiemindel/Downloads/blue-sphere-calibrate-b2.jpg', 1)))
    # print('TEST SPHERES')
    # print(testSpheres[0].toString())
    # exit()

    while True:
        img = cvs.takePhoto(ui=False)
        circles = cvs.findCircles(img, largestOnly=True, showImgs=True, live=True)
        spheres = cvs.findSpheres(circles)
        for sphere in spheres:
            print(sphere.toString())
        print()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # TODO: Write a node that runs this and outputs it
    # TODO: Support rosbag stuff

