from vrep import vrep # access all the VREP elements
import numpy as np
import glob
import time
import sys
import cv2
import matplotlib.pyplot as mlp

with np.load('B.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

i=0

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
    res, v1 = vrep.simxGetObjectHandle(clientID, 'camera_back', vrep.simx_opmode_oneshot_wait)
    print ('Getting first image')
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    
    #set vel
    vel=20 

    # Load the predefined dictionary
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    # Initialize the detector parameters using default values
    parameters =  cv2.aruco.DetectorParameters_create()


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
            
            # set dictionary size depending on the aruco marker selected
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

            # detector parameters can be set here (List of detection parameters[3])
            parameters = cv2.aruco.DetectorParameters_create()
            # parameters.adaptiveThreshConstant = 10

            # lists of ids and the corners belonging to each id
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # font for displaying text (below)
            font = cv2.FONT_HERSHEY_SIMPLEX
           
            # check if the ids list is not empty
            # if no check is added the code will crash
            if np.all(ids != None):

                # estimate pose of each marker and return the values
                # rvet and tvec-different from camera coefficients
                rvec, tvec ,_ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.2, mtx, dist)
                #(rvec-tvec).any() # get rid of that nasty numpy value array error

                for i in range(0, ids.size):
                    # draw axis for the aruco markers
                    cv2.aruco.drawAxis(img, mtx, dist, rvec[i], tvec[i], 0.1)

                # draw a square around the markers
                cv2.aruco.drawDetectedMarkers(img, corners)


                #code to show ids of the marker found

                # print('ids: \n',ids)
                # print('type(ids):',type(ids))
                choices=['back','front','left','right']
                new_ids=np.choose(ids,choices)
                # print('new ids: ',new_ids)
                strg = ''
                for i in range(0, new_ids.size):
                    strg += new_ids[i][0]+', '

                cv2.putText(img, "Id: " + strg, (10,64), font, 3, (0,255,0),2,cv2.LINE_AA)
                print('\nrotation vector: \n',rvec)
                print('\ntranslation vector: \n',tvec)


            else:
                # code to show 'No Ids' when no markers are found
                cv2.putText(img, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

            #     # project 3D points to image plane
            #     imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            #     img = draw(img,corners2,imgpts)
            #     print('\n\nimage',i)
            #     print('rotation vector: \n' ,rvecs)
            #     print('translation vector: \n' ,tvecs)
            

            # Detect the markers in the image
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

            # Detect the markers in the image
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
            img=cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
            # rejected=cv2.aruco.drawDetectedMarkers(copy_img, rejectedCandidates, (),(255,255,0))


            img=cv2.resize(img, (512,512))
            cv2.imshow('image',img)
            img_name = "{}.jpg".format(i)
            cv2.imwrite(img_name, img)    
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