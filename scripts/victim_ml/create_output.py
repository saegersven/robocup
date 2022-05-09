# Create output images from input csv data
# Output is 320x155 grayscale
# Dead victims are 0x7F. Living victims are 0xFF

import cv2
import os
import numpy as np

IMAGE_DIR = "../../ml_data/victims/victims"
NO_VICTIMS_DIR = "../../ml_data/victims/no_victims"
INPUT_OUTPUT_DIR = "../../ml_data/victims/input"
OUTPUT_IMAGE_DIR = "../../ml_data/victims/output"
NO_VICTIMS_OUTPUT_IMAGE_DIR = "../../ml_data/victims/output"
CSV_PATH = "out.csv"

with open(CSV_PATH, "r") as f:
	for row in f.readlines():
		cols = row.split(',')

		num_victims = int(cols[1])

		output_image = np.zeros((155, 320), dtype=np.uint8)

		if(num_victims != 0):

			victims = []

			for x in range(num_victims):
				i = 2 + x*5

				alive = (int(cols[i]) == 0)
				xmin = float(cols[i+1])
				ymin =  float(cols[i+2])
				xmax = float(cols[i+3])
				ymax = float(cols[i+4])

				x = (xmax + xmin) / 2
				y = (ymax + ymin) / 2
				radius = (xmax - xmin + ymax - ymin) / 4

				x /= 2
				y /= 2
				radius /= 2

				color = 0x7F
				if alive:
					color = 0xFF

				cv2.circle(output_image, (int(x), int(y)), int(radius), color, -1)

		cv2.imwrite(OUTPUT_IMAGE_DIR + "/" + cols[0], output_image)

for img in os.listdir(NO_VICTIMS_DIR):
	output_image = np.zeros((155, 320), dtype=np.uint8)
	cv2.imwrite(NO_VICTIMS_OUTPUT_IMAGE_DIR + "/" + img, output_image)