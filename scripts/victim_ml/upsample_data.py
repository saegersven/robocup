import numpy as np
import os
import cv2

INPUT_DIR = "../../ml_data/victims/victims"
OUTPUT_DIR = "../../ml_data/victims/victims_us"
INPUT_CSV = "out.csv"
OUTPUT_CSV = "out_us.csv"

input_width = 160
input_height = 120

rows_to_add = []

with open(INPUT_CSV, "r") as f:
	for row in f.readlines():
		rows_to_add.append(row)

		cols = row.split(',')
		img = cols[0]

		filename_f = img[0:-4] + "_f.png"
		new_row_f = f"{filename_f}, {cols[1]}"

		filename_b = img[0:-4] + "_b.png"
		new_row_b = row.replace(".png", "_b.png")

		for i in range(int(cols[1])):
			j = 2 + i*5
			new_xmin = input_width - float(cols[j + 3])
			new_xmax = input_width - float(cols[j + 1])

			new_row_f += f", {cols[j]}, {new_xmin}, {cols[j + 2]}, {new_xmax}, {cols[j + 4]}"

		rows_to_add.append(new_row_f)
		#rows_to_add.append(new_row_b)

		image = cv2.imread(INPUT_DIR + "/" + img, cv2.IMREAD_GRAYSCALE)
		print(img)
		image1 = np.flip(image, axis=1)

		# image_bright = image.copy()
		# for y in range(image.shape[0]):
		# 	for x in range(image.shape[1]):
		# 		image_bright[y, x] = np.clip(image_bright[y, x] + 60, 0, 255)

		cv2.imwrite(OUTPUT_DIR + "/" + img, image)
		cv2.imwrite(OUTPUT_DIR + "/" + filename_f, image1)
		#cv2.imwrite(OUTPUT_DIR + "/" + filename_b, image_bright)

with open(OUTPUT_CSV, "w") as f:
	for row in rows_to_add:
		f.write(row)