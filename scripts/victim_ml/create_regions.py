import cv2
import os
import numpy as np

# Input are 640x480 rgb images in folder and CSV file
# Output are 16 40x480 regions organized into folders

VICTIMS_IN = "../../ml_data/victims/victims"
NO_VICTIMS_IN = "../../ml_data/victims/no_victims"
CSV_PATH = "out.csv"

LIVING_VICTIM_OUT = "../../ml_data/victims/input_regions/living"
DEAD_VICTIM_OUT = "../../ml_data/victims/input_regions/dead"
NO_VICTIM_OUT = "../../ml_data/victims/input_regions/no_victim"

EMPTY_REGION = np.ones((480, 40), dtype=np.uint8) * 255

with open(CSV_PATH, "r") as f:
	for row in f.readlines():
		cols = row.split(',')

		image = cv2.imread(VICTIMS_IN + "/" + cols[0])
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		last_region = EMPTY_REGION.copy()
		next_region = image[0:480, 40:80]

		for x in range(16):
			region_xmin = x*40
			region_xmax = (x+1)*40
			region = image[0:480, region_xmin:region_xmax]

			has_victim = False
			dead = False

			for i in range(int(cols[1])):
				j = 2 + i*5
				xmin = float(cols[j + 1])
				xmax = float(cols[j + 3])
				dead = (int(cols[j]) == 1)

				if (xmin < region_xmin < xmax) or (xmin < region_xmax < xmax):
					has_victim = True
					break

			filename = cols[0][:-4] + "_" + str(x) + ".png"
			output_dir = NO_VICTIM_OUT

			composite_region = None

			if has_victim:
				composite_region = np.concatenate((last_region, region, next_region), axis=1)
				if dead:
					output_dir = DEAD_VICTIM_OUT
				else:
					output_dir = LIVING_VICTIM_OUT
			else:
				flipped_region = np.flip(region, axis=1)
				composite_region = np.concatenate((flipped_region, region.copy(), flipped_region.copy()), axis=1)

			print(output_dir + "/" + filename)

			cv2.imwrite(output_dir + "/" + filename, region)

			last_region = region
			if(x == 14):
				next_region = EMPTY_REGION.copy()
			else:
				next_region = image[0:480, (x+2)*40:(x+3)*40]

for img in os.listdir(NO_VICTIMS_IN):
	image = cv2.imread(NO_VICTIMS_IN + "/" + img)
	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	last_region = EMPTY_REGION.copy()
	next_region = image[0:480, 40:80]

	for x in range(16):
		region_xmin = x*40
		region_xmax = (x+1)*40
		region = image[0:480, region_xmin:region_xmax]

		composite_region = np.concatenate((last_region, region, next_region), axis=1)

		last_region = region
		if(x == 14):
			next_region = EMPTY_REGION.copy()
		else:
			next_region = image[0:480, (x+2)*40:(x+3)*40]

		cv2.imwrite(NO_VICTIM_OUT + "/" + img[:-4] + "_" + str(x) + ".png", region)