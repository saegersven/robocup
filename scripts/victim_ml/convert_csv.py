import csv

LABEL_CSV_PATH = r"E:\Robocup\robocup\ml_data\victims\victims_labelled\vott-csv-export\victims-export.csv"
OUTPUT_CSV_PATH = r"out.csv"
NO_VICTIMS_PATH = "../../ml_data/victims/no_victims"

CLASSES = ["living", "dead"]

l = {}

with open(LABEL_CSV_PATH, "r") as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	line_count = 0
	for row in csv_reader:
		if line_count == 0:
			line_count += 1
		else:
			d = {
				"xmin": row[1],
				"ymin": row[2],
				"xmax": row[3],
				"ymax": row[4],
				"class": row[5]
			}

			if row[0] in l:
				l[row[0]].append(d)
			else:
				l[row[0]] = [d]

with open(OUTPUT_CSV_PATH, "w") as csv_file:
	#csv_file.write("file, num_victims, class1, xmin1, ymin1, xmax1, ymax1, class2, xmin2, ymin2, xmax2, ymax2, class3, xmin3, ymin3, xmax3, ymax3\n")
	for filename in l:
		csv_file.write(f"{filename}, {len(l[filename])}")
		for i in range(len(l[filename])):
			csv_file.write(f", {CLASSES.index(l[filename][i]['class'])}, {l[filename][i]['xmin']}, {l[filename][i]['ymin']}, {l[filename][i]['xmax']}, {l[filename][i]['ymax']}")
		csv_file.write("\n")
