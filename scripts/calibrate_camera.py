import numpy as np
import cv2
import glob
import array
import time

image_folder = str(input("Input image folder: "))
out_file = str(input("Input output file: "))

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

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
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7,6), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)


cv2.destroyAllWindows()

print("Calibrating")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
    g.shape[::-1], None, None)

output_file = open(out_file, "wb")
float_array = array.array('f', [mtx[0][0], mtx[1][1], mtx[0, 2], mtx[2, 1],
    dist[0][0], dist[0][1], dist[0][2], dist[0][3], dist[0][4]])
float_array.tofile(output_file)
output_file.close()

print("Written camera parameters to output file")
print("Calculating error")

mean_error = 0
for i, objpoint in enumerate(objpoints):
    imgpoints2, _ = cv2.projectPoints(objpoint, rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error = mean_error + error

print(f"Total error: {mean_error/len(objpoints)}")