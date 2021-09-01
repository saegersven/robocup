import cv2
import numpy as np

def update(b, g, r):
	arr = np.array([[[b, g, r]]* 300] * 20, np.uint8)
	if not (b == 0 and g == 0 and r == 0):
		g_normalized = g / (b + g + r - min(b, g, r))
		saturation = max(b, g, r) - min(b, g, r)
		print(f"G(N):\t{g_normalized:.2}\tS:\t{saturation}")
	cv2.imshow("Window", arr)

def on_trackbar(v):
	b = cv2.getTrackbarPos("B", "Window")
	g = cv2.getTrackbarPos("G", "Window")
	r = cv2.getTrackbarPos("R", "Window")
	update(b, g, r)

cv2.namedWindow("Window")

cv2.createTrackbar("B", "Window", 0, 255, on_trackbar)
cv2.createTrackbar("G", "Window", 0, 255, on_trackbar)
cv2.createTrackbar("R", "Window", 0, 255, on_trackbar)

update(0, 0, 0)

cv2.waitKey()