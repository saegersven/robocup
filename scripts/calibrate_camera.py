import numpy as np
import cv2
import glob
import array
import time
import json

image_folder = "img" # str(input("Input image folder: "))
out_file = "test.json" # str(input("Input output file: "))

X = 5
Y = 7
# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((X*Y, 3), np.float32)
objp[:, :2] = np.mgrid[0:X, 0:Y].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

g = None

images = glob.glob(image_folder + '/*.jpg')
for fname in images:
    print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = gray
    cv2.imshow("g", g)
    cv2.waitKey(500)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (X,Y), None)
    # If found, add object points, image points (after refining them)
    if ret:
        print("Found")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (X,Y), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)


cv2.destroyAllWindows()

print("Calibrating")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
    g.shape[::-1], None, None)

out = {
    'fx': mtx[0][0],
    'fy': mtx[1][1],
    'cx': mtx[0][2],
    'cy': mtx[2][1],

    'k': [dist[0][0], dist[0][1], dist[0][2], dist[0][3], dist[0][4]]
}
j = json.dumps(out, indent=4)

output_file = open(out_file, 'w')
output_file.write(j)
output_file.close()

print("Written camera parameters to output file")
print("Calculating error")

mean_error = 0
for i, objpoint in enumerate(objpoints):
    imgpoints2, _ = cv2.projectPoints(objpoint, rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error = mean_error + error

print(f"Total error: {mean_error/len(objpoints)}")
