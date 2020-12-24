function sysCall_init()
    -- do some initialization here:
    startHandle=sim.getObjectHandle('StartConfiguration')
    goalHandle=sim.getObjectHandle('GoalConfiguration')
    robotHandle=sim.getObjectHandle('Center')
    collisionHandle=sim.getObjectHandle('Radius')
    environmentHandle=sim.getCollectionHandle('Environment')

    rightMotor=sim.getObjectHandle('Rmotor')
    leftMotor=sim.getObjectHandle('Lmotor')
    
    sensorHandle=sim.getObjectHandle('Proximity_sensor')
    refTime=sim.getSimulationTime()
    timeDelay=15
    
    d=0.6
    R=0.085
    Kp=1
    pos_on_path=0
    vn=0.8
    
    iter=0
    count=0

    t=simOMPL.createTask('t')
    simOMPL.setStateValidityCheckingResolution(t,0.01)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    ss={simOMPL.createStateSpace('2d',simOMPL.StateSpaceType.pose2d,startHandle,{-20,-20},{20,20},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setCollisionPairs(t,{collisionHandle,environmentHandle})

    startpos=sim.getObjectPosition(startHandle,-1)
    startorient=sim.getObjectOrientation(startHandle,-1)
    startpose={startpos[1],startpos[2],startorient[3]}
    simOMPL.setStartState(t,startpose)

    goalpos=sim.getObjectPosition(goalHandle,-1)
    goalorient=sim.getObjectOrientation(goalHandle,-1)
    goalpose={goalpos[1],goalpos[2],goalorient[3]}
    simOMPL.setGoalState(t,goalpose)

    r,path=simOMPL.compute(t,5,-1,800)

    pathHandle=sim.createPath(-1, nil, nil, nil)
    pc=#path/3
    for i=1,pc-1,1 do
    sim.insertPathCtrlPoints(pathHandle,0,i,1,{path[(i-1)*3+1],path[(i-1)*3+2],0,0,0,0,0,0,0,0,0})
    end
    path_length=sim.getPathLength(pathHandle)
end

function sysCall_actuation()
    -- put your actuation code here
    simTime=sim.getSimulationTime()
    
    position=sim.getPositionOnPath(pathHandle,pos_on_path)
    orientation=sim.getOrientationOnPath(pathHandle,pos_on_path)
    M=sim.getObjectMatrix(robotHandle,-1) --world frame
    sim.invertMatrix(M) --invert the rotation matrix
    path_pos=sim.multiplyVector(M,position) --robot frame

    distancia=math.sqrt((path_pos[1])^2+(path_pos[2])^2)
    angulo=math.atan2(path_pos[2],path_pos[1])
    if (pos_on_path < 1) then
    v=vn
    o=Kp*angulo
    else -- Stop at the end of the path
    v=0
    o=0
    end
    
    result,distance,detectedPoint,detectedObjectHandle=sim.readProximitySensor(sensorHandle)
    if result>0 and count<20 then
        wr=0
        wl=0
        count=count+1
        print(count)
    else
        wr=(v+d*o)/R
        wl=(v-d*o)/R
    end
    sim.setJointTargetVelocity(leftMotor,wl)
    sim.setJointTargetVelocity(rightMotor,wr)
    if (distancia < 0.8) then
    pos_on_path=pos_on_path+v*sim.getSimulationTimeStep()
    end
    
end

function sysCall_sensing()
    -- put your sensing code here
    
    result,distance,detectedPoint,detectedObjectHandle=sim.readProximitySensor(sensorHandle)
    
    if (result>0 and simTime-refTime>timeDelay) then

    sim.setJointTargetVelocity(leftMotor,0)
    sim.setJointTargetVelocity(rightMotor,0)
    print (sim.getObjectName(detectedObjectHandle)..' is detected!')
    print ('Distance is '..distance..' m')

    sim.setObjectParent(sim.getObjectHandle('Robot'),-1,1)
    pos=sim.getObjectPosition(robotHandle,-1)
    sim.setObjectPosition(startHandle,-1,pos)
    sim.setObjectParent(sim.getObjectHandle('Robot'),startHandle,1)
    sim.removeObject(sim.getObjectHandle('Path'))

    startpos=sim.getObjectPosition(startHandle,-1)
    startorient=sim.getObjectOrientation(startHandle,-1)
    startpose={startpos[1],startpos[2],startorient[3]}
    simOMPL.setStartState(t,startpose)

    goalpos=sim.getObjectPosition(goalHandle,-1)
    goalorient=sim.getObjectOrientation(goalHandle,-1)
    goalpose={goalpos[1],goalpos[2],goalorient[3]}
    simOMPL.setGoalState(t,goalpose)

    r,path=simOMPL.compute(t,5,-1,800)

    pathHandle=sim.createPath(-1, nil, nil, nil)
    pc=#path/3
    for i=1,pc-1,1 do
    sim.insertPathCtrlPoints(pathHandle,0,i,1,{path[(i-1)*3+1],path[(i-1)*3+2],0,0,0,0,0,0,0,0,0})
    end
    path_length=sim.getPathLength(pathHandle)
    
    pos_on_path=0
    refTime=sim.getSimulationTime()
        
    end

end

function sysCall_cleanup()
    -- do some clean-up here
end
