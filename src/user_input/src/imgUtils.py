import os
import cv2

def readImgsFromFolder(path):
	if os.path.isfile(path):
		img = cv2.imread(path)
		return [img], [path]
	elif os.path.isdir(path):
		paths = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f)) and f[0] != '.']
		imgs = []
		for f in paths:
			imgs.append(cv2.imread(os.path.join(path, f)))
		return imgs, paths
	else:
		return [], []
