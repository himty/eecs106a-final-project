import cv2
import numpy as np
import sys

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

if __name__ == '__main__':
	if len(sys.argv) == 0:
		print('Usage: thresholdGenerator.py <filename> <threshold>\nInput an image file, all of whose pixels are black, except for the pixels whose colors you\'d like to threshold.\nThreshold is the minimum value required for the item to be considered to be not a black pixel. 0 by default.')
		sys.exit(1)

	img = cv2.imread(sys.argv[1])
	threshold = 0
	if sys.argv[2]:
		threshold = int(sys.argv[2])

	print(f"Finding color thresholding bounds with minimum value {threshold} for masked file \"{sys.argv[1]}\"...")
	generateThresholds(img, threshold=threshold, verbose=True)
