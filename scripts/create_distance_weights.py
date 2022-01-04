import cv2
import math
import numpy as np

def distance_weight(d: float):
	return 2**(-(4.0 * d - 2.6)**2)

width = 80
height = 48

center_x = width / 2
center_y = height

m = np.empty((height, width), np.uint8)

for y in range(height):
	for x in range(width):
		dist = math.sqrt((y - center_y)**2 + (x - center_x)**2)
		if dist > 10 and dist < 40:
			dist -= 10
			dist /= 30
			weight = distance_weight(dist)
			print(weight)
			m[y][x] = weight * 255
		else:
			m[y][x] = 0

cv2.imshow("Out", m)

cv2.imwrite("runtime_data/distance_weights.png", m)

cv2.waitKey(5000)