import vrep
import math 
import time
import numpy as np


def to_rad(deg):
    return 2*math.pi*deg/360

def to_deg(rad):
    return rad*360/(2*math.pi)


# simulation config
ip = '127.0.0.1'
port = 19997
scene = './pioneer.ttm'
position_init = [0,0,to_rad(0)]
position_init1 = [3,3,to_rad(3)]


print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

if client_id!=-1:
    
    print ('Connected to remote API server on %s:%s' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    res, pioneer = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
    res, left_motor = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    res, right_motor = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
    res, wall = vrep.simxGetObjectHandle(client_id, '20cmHighWall50cm', vrep.simx_opmode_oneshot_wait)
    
    # For Sensors
    tmp=np.zeros(16) #For robot position
    tmp2=np.zeros(16) # For Goal position
    sensor_handles=np.zeros(16)
    detectStatus = np.zeros(16)
    
    
    # Reading data for sensors
    for i in range(1,17) : 
        res , sensor_handle = vrep.simxGetObjectHandle(client_id, "Pioneer_p3dx_ultrasonicSensor" + str(i), vrep.simx_opmode_blocking)
        sensor_handles[i-1] = sensor_handle
        res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(client_id, sensor_handle, vrep.simx_opmode_streaming)
        
         
    #intial values of Robot Speed
    v0 = 1 # Linear Speed of the robot
    w0 = 0 # Angular Speed of the robot
    k1=0.1
    k2=0.1

    
    # Braitenberg Algorithm Parameters

    maxDetectionRadius = 0.3
    SafetyDist = 0.2
    braitenbergL = np.array([-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    braitenbergR = np.array([-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])    
    simStatusCheck = vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot)
        
    # Other parameters 
    state = 0
    Robot_position = position_init 
    Goal_position = position_init1
    thresold_distance = 0.5
    

    # Goal position
    res, tmp2 = vrep.simxGetObjectPosition(client_id,wall,-1, vrep.simx_opmode_oneshot_wait)
    Goal_position[0]= tmp2[0] 
    Goal_position[1]= tmp2[1] 
    

    # Main programm
    continue_running = True
    while(continue_running):
        

        # Detecting obstacles
        for i in range(1,17) : 
            res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(client_id, int(sensor_handles[i-1]), vrep.simx_opmode_buffer)
            distObject = math.sqrt(math.pow(detectedPoint[0], 2) + math.pow(detectedPoint[1], 2) + math.pow(detectedPoint[2], 2)) # Calculate distance to obstacle relative to each sensor 
            if (detectionState == True) and (distObject < maxDetectionRadius):
                if (distObject < SafetyDist): 
                    distObject = SafetyDist
                detectStatus[i-1] = 1-((distObject - SafetyDist)/(maxDetectionRadius - SafetyDist))
                state = 1
            else:
                detectStatus[i-1] = 0



        # Robot position 
        res, tmp = vrep.simxGetObjectPosition(client_id,pioneer,-1, vrep.simx_opmode_oneshot_wait)
        Robot_position[0]= tmp[0] #X_r
        Robot_position[1]= tmp[1] #Y_r
        res, tmp = vrep.simxGetObjectOrientation(client_id, pioneer, -1, vrep.simx_opmode_oneshot_wait)
        Robot_position[2] = tmp[2] # en radian  


        #Distance to Goal 
        d = math.sqrt(math.pow(Goal_position[0] - Robot_position[0],2) + math.pow(Goal_position[1]- Robot_position[1],2))

        Goal_teta = math.atan((Goal_position[1] - Robot_position[1])/(Goal_position[0] - Robot_position[0]))
        delta_teta = Robot_position[2] - Goal_teta
        w0  = -delta_teta

        # Wheel speeds if no obstacle is near the robot 
        
        v_left = ((v0/(2*k1) - w0/(2*k2)))
        v_right = ((v0/(2*k2)) + w0/(2*k1))
        

        # Wheels Velocity if there is an obstacle near the robot
        if  state == 1 : 
            # adjust wheel speeds 
            v_left = v0
            v_right = v0
            # braitenberg vehicle
            for i in range(1,17):
                v_left = v_left + braitenbergL[i-1] * detectStatus[i-1]
                v_right = v_right + braitenbergR[i-1] * detectStatus[i-1]
            state = 0
    

        res = vrep.simxSetJointTargetVelocity(client_id, left_motor, v_left, vrep.simx_opmode_oneshot)
        res = vrep.simxSetJointTargetVelocity(client_id, right_motor, v_right, vrep.simx_opmode_oneshot)  


        # cancel the speed and stop the simulation if the robot has reached the objective 
        if (d<=thresold_distance):
            res = vrep.simxSetJointTargetVelocity(client_id, left_motor, 0, vrep.simx_opmode_oneshot)
            res = vrep.simxSetJointTargetVelocity(client_id, right_motor, 0, vrep.simx_opmode_oneshot)  
            print("Robot reached the goal ")  
            continue_running = False        
    
    
    #Terminate
    vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(client_id)
    
else:
    print('Unable to connect to %s:%s' % (ip, port))
