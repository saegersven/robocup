import cv2
import os

def crop_and_save(img, dest):
	image = cv2.imread(img)
	cropped = image[4:14, 5:47]
	#cv2.imshow("Cropped", cropped)
	#cv2.waitKey(5000)
	#exit()
	cv2.imwrite(dest, cropped)

silver_data_dir = "../../ml_data/silver/silver_rois_sorted"
no_silver_data_dir = "../../ml_data/linefollowing/linefollowing_rois_sorted"

silver_dest_dir = "../../ml_data/silver_cropped/silver"
no_silver_dest_dir = "../../ml_data/silver_cropped/no_silver"

counter = 0

for img in os.listdir(silver_data_dir):
	crop_and_save(silver_data_dir + "/" + img, silver_dest_dir + "/" + str(counter) + ".png")
	counter += 1

counter2 = 0

for img in os.listdir(no_silver_data_dir):
	crop_and_save(no_silver_data_dir + "/" + img, no_silver_dest_dir + "/" + str(counter2) + ".png")
	counter2 += 1

print(f"Done. Processed {counter + counter2} images")