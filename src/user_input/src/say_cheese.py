import cv2
import time

if __name__ == '__main__':
	cap = cv2.VideoCapture(0)

	while True:
		ret, frame = cap.read()
		if ret:
			cv2.imshow('img', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				cv2.imwrite('cv2_capture_' + str(time.time()) + '.png', frame)
				break
