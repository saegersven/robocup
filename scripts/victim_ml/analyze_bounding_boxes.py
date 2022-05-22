import cv2
import matplotlib.pyplot as plt
import numpy as np

plt.style.use('seaborn-whitegrid')

CSV_PATH = "out.csv"

y_coord_data = []
area_data = []

with open(CSV_PATH, "r") as f:
	for row in f.readlines():
		cols = row.split(',')

		for i in range(int(cols[1])):
			j = 2 + i*5
			xmin = float(cols[j + 1])
			ymin = float(cols[j + 2])
			xmax = float(cols[j + 3])
			ymax = float(cols[j + 4])
			x_v = (xmin + xmax) / 2
			y_v = (ymin + ymax) / 2
			w_v = xmax - xmin
			h_v = ymax - ymin

			area = w_v * h_v
			y_coord_data.append(y_v)
			area_data.append(area)

y_coord_data = np.array(y_coord_data)
area_data = np.array(area_data)

theta = np.polyfit(y_coord_data, area_data, 1)
y_line = theta[1] + theta[0] * y_coord_data

tolerance = 8000

plt.scatter(y_coord_data, area_data)
plt.plot(y_coord_data, y_line, 'r')
plt.plot(y_coord_data, y_line - tolerance, 'r')
plt.plot(y_coord_data, y_line + tolerance, 'r')
plt.show()