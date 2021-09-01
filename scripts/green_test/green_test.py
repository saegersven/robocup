import cv2
import numpy as np
import math
import pathlib

def deg_to_rad(num):
	return num * 0.01745329251
def rad_to_deg(num):
	return num / 0.01745329251

def debug(text):
	if __name__ == "__main__":
		print(text)
# data = list(pathlib.Path("../../ml/green/data").glob("*/*.jpg"))
# err = 0
# max_count = 500
# counter = 0
# for filename in data:
# 	if(counter == max_count):
# 		break

def green_test(image):
	#image = cv2.imread(filename.__str__())
	image_debug = image.copy()
	#image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# image = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
	# image = cv2.medianBlur(image, 3)

	# green = cv2.inRange(image_hsv, (40, 50, 15), (70, 255, 255))
	green = np.zeros((image.shape[0], image.shape[1]), np.uint8)
	for x in range(image.shape[1]):
		for y in range(image.shape[0]):
			m = min(image[y][x][0], image[y][x][1], image[y][x][2])
			s = float(image[y][x][0]) + float(image[y][x][2]) + float(image[y][x][2]) - m
			if(s == 0.0):
				continue
			ratio = float(image[y][x][1]) / s

			if(ratio > 0.55 and image[y][x][1] > 40):
				green[y][x] = 255
				# print(f"{ratio}\t{image[y][x][1]}")

	#green = cv2.inRange(image, (0, 60, 0), (40, 255, 40))

	black = cv2.inRange(image, (0, 0, 0), (50, 50, 50))

	kernel = np.ones((3, 3), np.uint8)
	green = cv2.erode(green, kernel, iterations=1)
	green = cv2.dilate(green, kernel, iterations=1)

	cv2.imshow("Green", green)
	cv2.imshow("Black", black)

	contours, hierarchy = cv2.findContours(green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# num_contours = len(contours)

		# for i in range(len(contours)):
		# 	r = cv2.minAreaRect(contours[i])

		# 	if(r[1][0] * r[1][1] < 20.0):
		# 		num_contours = num_contours - 1

		# if(num_contours != 1):
		# 	debug(f"'{filename.__str__()}' wrong ({num_contours})")
		# 	cv2.imshow("Green", green)
		# 	cv2.imshow("Image", image)
		# 	cv2.waitKey(1000)
		# 	err = err + 1
		# counter = counter + 1

	#debug(f"{err} wrong in {min(len(data), max_count)} images ({err / min(len(data), max_count) * 100.0}%)")

	green_points = 0

	#bottom_center = (image.cols / 2, image.rows - 1)

	left = False
	right = False

	for i in range(len(contours)):
		r = cv2.minAreaRect(contours[i])

		if(r[1][0] * r[1][1] < 40.0):
			continue
		# Debug
		box = cv2.boxPoints(r) # cv2.boxPoints(rect) for OpenCV 3.x
		box = np.int0(box)
		cv2.drawContours(image_debug, [box], 0, (200,0,0), 2)
		cv2.imshow("Debug", image_debug)

		# angle = deg_to_rad(r[2])
		# if(angle > deg_to_rad(45.0)):
		# 	angle = angle - deg_to_rad(90.0)
		# debug(f"Angle:\t{rad_to_deg(angle)}")
		# cx = 0.0
		# cy = 0.0
		# black_mask = 0
		# for x in range(4):
		# 	angle = angle + deg_to_rad(90.0)
		# 	debug(rad_to_deg(angle))
		# 	black_counter = 0
		# 	for out in range(10):
		# 		cx = r[0][0] + math.cos(angle) * (r[1][0] * 0.6 + out * 0.05 * r[1][0])
		# 		cy = r[0][1] + math.sin(angle) * (r[1][1] * 0.6 + out * 0.05 * r[1][1])
		# 		c = (0, 100, 0)
		# 		bl = image[int(cy)][int(cx)][0]
		# 		gr = image[int(cy)][int(cx)][1]
		# 		re = image[int(cy)][int(cx)][2]
		# 		lightness = (bl + gr + re) / 3
		# 		saturation = max(bl, gr, re) - min(bl, gr, re)
		# 		if(lightness < 20 and saturation < 20):
		# 			black_counter = black_counter + 1
		# 			c = (0, 0, 255)

		# 		cv2.circle(image_debug, (int(cx), int(cy)), 1, c, 1)

		# 	if(black_counter > 3):
		# 		black_mask = black_mask | (1 << x)
		# 		debug(f"Line {x}, {black_mask}")

		# if(i == 0):
		# 	continue

		initial_angle = deg_to_rad(r[2])
		if(initial_angle > deg_to_rad(45.0)):
			initial_angle = initial_angle - deg_to_rad(90.0)
		angle = initial_angle
		#cv2.line(image_debug, (int(r[0][0]), int(r[0][1])), (int(r[0][0] + math.sin(angle) * 15), int(r[0][1] - math.cos(angle) * 15)), (0, 255, 255), 2)

		debug(f"Initial Angle:\t{rad_to_deg(initial_angle)}")
		black_points = []
		for x in range(30):
			angle = deg_to_rad(360.0 / 30.0 * x)
			if(angle > deg_to_rad(180.0)):
				angle = angle - deg_to_rad(360.0)
			cx = r[0][0] + math.sin(angle) * 15
			cy = r[0][1] - math.cos(angle) * 15
			c = (0, 255, 0)

			out_of_range = False

			for out in range(6):
				cx = cx + math.sin(angle) * out * 0.5
				cy = cy - math.cos(angle) * out * 0.5

				if cx >= image.shape[0] or cy >= image.shape[0] or cx < 0 or cy < 0:
					out_of_range = True
					continue

				bl = image[int(cy)][int(cx)][0]
				gr = image[int(cy)][int(cx)][1]
				re = image[int(cy)][int(cx)][2]
				# lightness = int((float(bl) + float(gr) + float(re)) / float(3))
				# saturation = max(bl, gr, re) - min(bl, gr, re)
				# if(lightness < 50 and saturation < 20):
				if(black[int(cy)][int(cx)] > 0):
					# print(f"{lightness}\t{saturation}")
					black_points.append((cx, cy))
					debug(rad_to_deg(angle))
					c = (0, 0, 255)
					break
			if not out_of_range:
				image_debug[int(cy)][int(cx)] = c

		if(len(black_points) > 5):
			mean_point = (0, 0)

			for point in black_points:
				mean_point = (mean_point[0] + point[0], mean_point[1] + point[1])

			mean_point = (mean_point[0] / len(black_points), mean_point[1] / len(black_points))

			image_debug[int(mean_point[1]), int(mean_point[0])] = (100, 255, 100)

			mean_angle = np.arctan2(r[0][1] - mean_point[1], r[0][0] - mean_point[0]) - deg_to_rad(90.0) - initial_angle

			if(mean_angle < 0.0):
				mean_angle = mean_angle + deg_to_rad(360.0)

			cv2.line(image_debug, (int(r[0][0]), int(r[0][1])),
				(int(r[0][0] + math.sin(mean_angle + initial_angle) * 20),
					int(r[0][1] - math.cos(mean_angle + initial_angle) * 20)), (255, 0, 255), 1)

			debug(f"Mean angle:\t{rad_to_deg(mean_angle)} ({len(black_points)})")

			#cv2.circle(image_debug, (int(r[0][0]), int(r[0][1])), abs(int(rad_to_deg(mean_angle) / 7.0)), (255, 0, 0), 1)

			if(mean_angle > 0.0 and abs(mean_angle) < deg_to_rad(90.0)):
				left = True
			elif(mean_angle <= 0.0 and abs(mean_angle) < deg_to_rad(90.0)):
				right = True
			print(f"{i}:\t{rad_to_deg(mean_angle)}")

		# debug(black_mask)
		# if(black_mask == 12):
		# 	left = True
		# elif(black_mask == 6):
		# 	right = True


	debug(f"{left}\t{right}")
	cv2.imshow("Image", image_debug)
	return (left, right)

if __name__ == "__main__":
	image = cv2.imread("test_images/6.jpg")
	decision = green_test(image)

	debug(decision)

	while cv2.getWindowProperty("Image", cv2.WND_PROP_VISIBLE) >= 1:
		keyCode = cv2.waitKey(10)
		if(keyCode & 0xFF) == ord("q"):
			cv2.destroyAllWindows()
			break