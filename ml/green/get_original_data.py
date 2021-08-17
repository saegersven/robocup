import os
import subprocess

folders = ["left", "right"]

subprocess.call("mkdir original_data", shell=False)

for folder in folders:
	subprocess.call(f"cd original_data && mkdir {folder} && cd ..", shell=False)
	for fileL in os.listdir(f"data/{folder}"):
		if(len(fileL) == 4 or len(fileL) == 5):
			subprocess.call(f"copy data\\{folder}\\{fileL} original_data\\{folder}\\{fileL}", shell=False)