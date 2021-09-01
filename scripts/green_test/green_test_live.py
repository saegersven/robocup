import green_test
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (80, 48)
camera.framerate = 100
rawCapture = PiRGBArray(camera, size=(80, 48))

# cap = cv2.VideoCapture(0)
# cap.set(3, 80)
# cap.set(4, 48)
# if not cap.isOpened():
# 	print("Could not open camera")
# 	exit()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# ret, frame = cap.read()

	# if not ret:
	# 	print("Could not receive frame")
	# 	break

	image = frame.array

	# cv2.imwrite("pic.png", image)

	decision = green_test.green_test(image)

	print(decision)

	rawCapture.truncate(0)

	if cv2.waitKey(1) == ord("q"):
		break

# cap.release()
cv2.destroyAllWindows()