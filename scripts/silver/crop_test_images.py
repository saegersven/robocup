import cv2
import numpy as np
import glob

def scalar_difference(a, b):
	d = 0
	for x in range(a.shape[1]):
		for y in range(a.shape[0]):
			d += abs(int(a[y][x][0]) - b[y][x][0])
			d += abs(int(a[y][x][1]) - b[y][x][1])
			d += abs(int(a[y][x][2]) - b[y][x][2])
	return d

folder = "E:\\Robocup\\robocup\\ml\\silver\\test_images_silver\\t"#str(input("Input folder name: "))
res = (80, 48)

images = glob.glob(folder + "/*.png")

cut_images = []

print(len(images))

for fname in images:
	if "cut" in fname or "diff" in fname:
		continue
	img = cv2.imread(fname)

	# Cut out part
	cut = img[27:33,29:52]
	cv2.imwrite(fname + "_cut.png", cut)
	cut_images.append(cut)

average_cut = cut_images[0]
for i in range(1, len(cut_images)):
	average_cut = cv2.addWeighted(cut_images[i], 0.5, average_cut, 0.5, 0.0)

cv2.imwrite(folder + "/average_cut.png", average_cut)

a = cv2.imread(folder + "/diff_test.png")

diff = scalar_difference(a, average_cut)
print(diff)