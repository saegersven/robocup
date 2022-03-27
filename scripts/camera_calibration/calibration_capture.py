import cv2

cam_id = int(input("Input camera Id: "))
folder = str(input("Input output folder: "))

cam = cv2.VideoCapture(cam_id, cv2.CAP_V4L)

counter = 0

while True:
	ret, img = cam.read()
	img = cv2.flip(img, -1)
	cv2.imshow("Captured image", img)

	w = cv2.waitKey(1)
	if w == ord('c'):
		cv2.imwrite(folder + "/" + str(counter) + ".jpg", img)
		print("Captured")
		counter = counter + 1
	elif w == ord('q'):
		break

cv2.destroyAllWindows()