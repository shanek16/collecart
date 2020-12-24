from vrep import vrep # access all the VREP elements
import numpy as np
import glob
import time
import sys
import cv2
import matplotlib.pyplot as mlp
i=0
#-----------------------------pose_estimation setting---------------
def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 70, 1)
objp = np.zeros((6*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

#----------------------------------vrep--------------------------------
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection

if clientID!=-1:
    print ("Connected to remote API server")
    #motor and steer
    err_code,fl_motor_handle = vrep.simxGetObjectHandle(clientID,"fl_joint", vrep.simx_opmode_blocking)
    err_code,fr_motor_handle = vrep.simxGetObjectHandle(clientID,"fr_joint", vrep.simx_opmode_blocking)
    err_code,bl_motor_handle = vrep.simxGetObjectHandle(clientID,"bl_joint", vrep.simx_opmode_blocking)
    err_code,br_motor_handle = vrep.simxGetObjectHandle(clientID,"br_joint", vrep.simx_opmode_blocking)

    err_code,bl_joint=vrep.simxGetObjectHandle(clientID,"bl_steer", vrep.simx_opmode_blocking)
    err_code,fl_joint=vrep.simxGetObjectHandle(clientID,"fl_steer", vrep.simx_opmode_blocking)
    err_code,br_joint=vrep.simxGetObjectHandle(clientID,"br_steer", vrep.simx_opmode_blocking)
    err_code,fr_joint=vrep.simxGetObjectHandle(clientID,"fr_steer", vrep.simx_opmode_blocking)

    t = time.time() #record the initial time

    #camera
    print ('Vision Sensor object handling')
    res, v1 = vrep.simxGetObjectHandle(clientID, 'camera_front', vrep.simx_opmode_oneshot_wait)
    print ('Getting first image')
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    
    #set vel
    vel=-10 

    while (vrep.simxGetConnectionId(clientID) != -1):
        #pin steer
        err_code = vrep.simxSetJointTargetPosition(clientID,bl_joint,0,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetPosition(clientID,fl_joint,0,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetPosition(clientID,br_joint,0,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetPosition(clientID,fr_joint,0,vrep.simx_opmode_streaming)
        #motor control
        err_code = vrep.simxSetJointTargetVelocity(clientID,fl_motor_handle,vel,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,fr_motor_handle,vel,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,bl_motor_handle,vel,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,br_motor_handle,vel,vrep.simx_opmode_streaming)
        # time.sleep(0.2)
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            img=cv2.flip(img,0)
            #---------pose_estimation-------------------
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (6,6),None)

            if ret == True:
                i=i+1
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (6,6), corners2,ret)
                cv2.imshow('img',img)
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
                np.savez('B',ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
                cv2.waitKey(500)
 
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == vrep.simx_return_novalue_flag:
            print ("no image yet")
            pass
        else:
          print (err)
else:
  print ("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)

cv2.destroyAllWindows()