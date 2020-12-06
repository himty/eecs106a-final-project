import cv2
import numpy as np
import sys
import os

def generateThresholds(bgrImg, threshold=0, verbose=False):
	hsvImg = cv2.cvtColor(bgrImg, cv2.COLOR_BGR2HSV)
	# Filter out black pixels
	height, width, numChannels = np.shape(hsvImg)
	flattened = np.reshape(hsvImg, (height * width, numChannels))
	pixels = list(filter(lambda x: x[2] > threshold, flattened))
	# Find the maximum and minimum h, s, and v values in the image
	hMin = min(pixels, key=lambda x: x[0])[0]
	hMax = max(pixels, key=lambda x: x[0])[0]
	sMin = min(pixels, key=lambda x: x[1])[1]
	sMax = max(pixels, key=lambda x: x[1])[1]
	vMin = min(pixels, key=lambda x: x[2])[2]
	vMax = max(pixels, key=lambda x: x[2])[2]
	if verbose: print(f"Min: ({hMin}, {sMin}, {vMin})\nMax: ({hMax}, {sMax}, {vMax})")
	return ((hMin, sMin, vMin), (hMax, sMax, vMax))

def generateThresholdsMany(bgrImgs, threshold=0, verbose=False):
	if not bgrImgs or len(bgrImgs) == 0:
		return None, None

	hMin = np.inf
	hMax = -1
	sMin = np.inf
	sMax = -1
	vMin = np.inf
	vMax = -1

	for bgrImg in bgrImgs:
		if verbose: print("Running on a new image...")
		hsvMin, hsvMax = generateThresholds(bgrImg, threshold, verbose=verbose)

		if hsvMin[0] < hMin:
			hMin = hsvMin[0]
		if hsvMin[1] < sMin:
			sMin = hsvMin[1]
		if hsvMin[2] < vMin:
			vMin = hsvMin[2]

		if hsvMax[0] > hMax:
			hMax = hsvMax[0]
		if hsvMax[1] > sMax:
			sMax = hsvMax[1]
		if hsvMax[2] > vMax:
			vMax = hsvMax[2]

	if verbose: print(f"\nMin: ({hMin}, {sMin}, {vMin})\nMax: ({hMax}, {sMax}, {vMax})")
	return ((hMin, sMin, vMin), (hMax, sMax, vMax))

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print('Usage: thresholdGenerator.py <filename> <threshold>\nInput an image file, all of whose pixels are black, except for the pixels whose colors you\'d like to threshold.\nThreshold is the minimum value required for the item to be considered to be not a black pixel. 0 by default.')
		sys.exit(1)

	threshold = 0
	if len(sys.argv) >= 3 and sys.argv[2]:
		threshold = int(sys.argv[2])

	filename = sys.argv[1]
	if os.path.isfile(filename):
		img = cv2.imread(filename)
		print(f"Finding color thresholding bounds with minimum value {threshold} for masked file \"{filename}\"...")
		generateThresholds(img, threshold=threshold, verbose=True)
	elif os.path.isdir(filename):
		filenames = [os.path.join(filename, f) for f in os.listdir(filename) if os.path.isfile(os.path.join(filename, f)) and f[0] != '.']
		imgs = []
		for f in filenames:
			imgs.append(cv2.imread(f))
		print(f"Finding color thresholding bounds with minimum value {threshold} for the following masked files:")
		for f in filenames:
			print(f)
		print()
		generateThresholdsMany(imgs, threshold=threshold, verbose=True)
	else:
		print(f"Failed to threshold. The inputted path is neither a file nor a directory.")
