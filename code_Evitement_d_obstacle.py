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


print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

if client_id!=-1:
    
    print ('Connected to remote API server on %s:%s' % (ip, port))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    res, pioneer = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
    res, left_motor = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    res, right_motor = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
    
    # For Sensors
    sensor_handles=np.zeros(16)
    sensors_handles=np.zeros(16)
    detectStatus = np.zeros(16)
    
    
    # Reading data for sensors
    for i in range(1,17) : 
        res , sensor_handle = vrep.simxGetObjectHandle(client_id, "Pioneer_p3dx_ultrasonicSensor" + str(i), vrep.simx_opmode_blocking)
        sensor_handles[i-1] = sensor_handle
        res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(client_id, sensor_handle, vrep.simx_opmode_streaming)
        
        
    #intial values of Robot Speed
    
    v0 = 1.5
    v_l = 0
    v_r = 0
    
    # Braitenberg Algorithm Parameters
    
    maxDetectionRadius = 0.5 
    minSafetyDist = 0.2
    braitenbergL = np.array([-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    braitenbergR = np.array([-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])    
    simStatusCheck = vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot)
        
    continue_running = True
    while(continue_running):
        
        for i in range(1,17) : 
            res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(client_id, int(sensor_handles[i-1]), vrep.simx_opmode_buffer)
            distToObject = math.sqrt(math.pow(detectedPoint[0], 2) + math.pow(detectedPoint[1], 2) + math.pow(detectedPoint[2], 2)) # Calculate distance to obstacle relative to each sensor 
            
            
            if (detectionState == True) and (distToObject < maxDetectionRadius):
                if (distToObject < minSafetyDist): 
                    distToObject = minSafetyDist
                detectStatus[i-1] = 1-((distToObject - minSafetyDist)/(maxDetectionRadius - minSafetyDist))
            else:
                detectStatus[i-1] = 0
        
        v_l = v0
        v_r = v0
    
        for i in range(1,17):
            v_l = v_l + braitenbergL[i-1] * detectStatus[i-1]
            v_r = v_r + braitenbergR[i-1] * detectStatus[i-1]
    
    
        res = vrep.simxSetJointTargetVelocity(client_id, left_motor, v_l, vrep.simx_opmode_oneshot)
        res = vrep.simxSetJointTargetVelocity(client_id, right_motor, v_r, vrep.simx_opmode_oneshot)            

    #Terminate
    vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(client_id)
    

else:
    print('Unable to connect to %s:%s' % (ip, port))
