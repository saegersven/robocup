import cv2
import glob

num_silver = 0

for i, filename in enumerate(glob.glob("test_images/*.jpg")):
	img = cv2.imread(filename)

	MIN_VALUE = 100
	MIN_C_VALUE = 200
	MIN_RATIO = 0.6

	COORD_C = (35, 33)
	SIZE_C = (4, 13)

	COORD_L = (34, 26)
	SIZE_L = (6, 6)

	COORD_R = (34, 47)
	SIZE_R = (6, 6)

	roi_L = img[COORD_L[0]:COORD_L[0]+SIZE_L[0], COORD_L[1]:COORD_L[1]+SIZE_L[1]]
	roi_R = img[COORD_R[0]:COORD_R[0]+SIZE_R[0], COORD_R[1]:COORD_R[1]+SIZE_R[1]]
	roi_C = img[COORD_C[0]:COORD_C[0]+SIZE_C[0], COORD_C[1]:COORD_C[1]+SIZE_C[1]]

	# cv2.imshow("ROI L", roi_L)
	# cv2.imshow("ROI R", roi_R)
	# cv2.imshow("ROI C", roi_C)
	# cv2.waitKey(5000)

	average_L = roi_L.mean(axis=0).mean(axis=0)
	average_R = roi_R.mean(axis=0).mean(axis=0)
	v_C = roi_C.mean(axis=0).mean(axis=0).mean(axis=0)

	r_L = average_L[2] / (average_L[1] + average_L[0])
	v_L = average_L.mean(axis=0)

	r_R = average_R[2] / (average_R[1] + average_R[0])
	v_R = average_R.mean(axis=0)

	print(i)
	print(f"Left:  \t{r_L:.2f}\t{v_L:.2f}")
	print(f"Right: \t{r_R:.2f}\t{v_R:.2f}")
	print(f"Center:\t{v_C:.2f}")

	if(r_L >= MIN_RATIO and v_L >= MIN_VALUE and r_R >= MIN_RATIO and v_R >= MIN_VALUE and v_C > MIN_C_VALUE):
		print("SILVER!")
		num_silver += 1

print(num_silver)