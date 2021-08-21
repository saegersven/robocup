import cv2

cam_id = int(input("Input camera Id: "))
folder = str(input("Input output folder: "))

cam = cv2.VideoCapture(cam_id)

counter = 0

while True:
	cmd = str(input("Press enter to capture, type anything to abort: "))
	if(len(cmd) != 0):
		break
	ret, img = cam.read()
	cv2.imshow("Captured image", img)
	cv2.imwrite(folder + "/" + str(counter) + ".jpg", img)

	counter = counter + 1
	if cv2.waitKey(1) == 27:
		break

cv2.destroyAllWindows()