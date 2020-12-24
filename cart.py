from vrep import vrep # access all the VREP elements
import numpy as np
import glob
import time
import sys
import cv2
import matplotlib.pyplot as mlp

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection

if clientID!=-1:
    print ("Connected to remote API server")
    err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"fl_joint", vrep.simx_opmode_blocking)
    err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"fr_joint", vrep.simx_opmode_blocking)

    err_code,ps_handle = vrep.simxGetObjectHandle(clientID,"us_f2", vrep.simx_opmode_blocking)
    err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ps_handle,vrep.simx_opmode_streaming)

    t = time.time() #record the initial time
    print ('Vision Sensor object handling')
    res, v1 = vrep.simxGetObjectHandle(clientID, 'camera_front', vrep.simx_opmode_oneshot_wait)
    print ('Getting first image')
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    while (vrep.simxGetConnectionId(clientID) != -1):
        sensor_val = np.linalg.norm(detectedPoint)
        if sensor_val < 0.2 and sensor_val>0.01:
            l_steer = -1/sensor_val
        else:
            l_steer = 1.0
        err_code = vrep.simxSetJointTargetVelocity(clientID,l_motor_handle,l_steer,vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID,r_motor_handle,1.0,vrep.simx_opmode_streaming)
        # time.sleep(0.2)
        err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ps_handle,vrep.simx_opmode_buffer)

        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            print ("image OK!!!")
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            img=cv2.flip(img,0)
            cv2.imshow('image',img)
            img_name = "./img/{}.jpg".format(time.time())
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