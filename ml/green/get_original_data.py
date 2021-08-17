import os

folders = ["left", "right"]

os.system("mkdir original_data")

for folder in folders:
	os.system(f"cd original_data && mkdir {folder} && cd ..")
	for fileL in os.listdir(f"data/{folder}"):
		if(len(fileL) == 4 or len(fileL) == 5):
			os.system(f"copy data\\{folder}\\{fileL} original_data\\{folder}\\{fileL}")