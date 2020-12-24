import cv2
import numpy as np
import glob

# Load previously saved data
#with np.load('B.npz') as X:
#    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

detected=0
not_detected=0

for fname in glob.glob('./img/*.jpg'):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (6,6),None)

    if ret == True:
        detected=detected+1
        print('detected')
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Find the rotation and translation vectors.
        _, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        img = draw(img,corners2,imgpts)
        cv2.imshow('img',img)
        if cv2.waitKey(0) & 0xff == ord('q'):
            cv2.destroyAllWindows()
            raise StopIteration   
    else:
        print('not detected')
        not_detected=not_detected+1
        cv2.imshow('img',gray)
        if cv2.waitKey(0) & 0xff == ord('q'):
            cv2.destroyAllWindows()
            raise StopIteration
print('detected: %d'%detected)
print('not detected: %d'%not_detected)
cv2.destroyAllWindows()