import cv2
import time
import GPIO

cap = cv2.VideoCapture(0)
img_cnt = 0
while(True):
	_, frame = cap.read()
	cv2.imshow('frame', frame)
	if cv2.waitKey(1) & 0xFF == ord('c'):
		path = "/home/pi/Desktop/imgs/" + str(time.time()) + ".png"
		cv2.imwrite(path, frame)
		print("saved image", img_cnt)
		img_cnt = img_cnt + 1
		time.sleep(0.5)
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()