try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')
    exit(0)


import numpy as np
import math

theta_desired,x_state_desired,theta_dot_desired,x_state_dot_desired = 0.0,0.0,0.0,0.0

x_state_init,theta_init = 0.0,0.0


speed_1,speed_2 = tuple(0 for x in range(2))


k = np.array([[0.7071,1.12,-11.4786,-1.7710],[0.7071,1.12,-11.4786,-1.7710]])  #put the values of k here
X = np.array([[0.0],[0.0],[0.0],[0.0]])
u = np.array([[0.0],[0.0]])


def balance(clientID):
    ret = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
    ret,battery=vrep.simxGetObjectHandle(clientID,'battery',vrep.simx_opmode_blocking)
    ret,sens=vrep.simxGetObjectHandle(clientID,'stm32f4_discovery',vrep.simx_opmode_blocking)
    ret,motor1=vrep.simxGetObjectHandle(clientID,'motor_right_model',vrep.simx_opmode_blocking)
    ret,motor2=vrep.simxGetObjectHandle(clientID,'motor_left_model',vrep.simx_opmode_blocking)

    global theta_init,speed_1,speed_2
    global x_state_init
    global X,k,u
    global theta_desired,x_state_desired,theta_dot_desired,x_state_dot_desired
    
    while(True):
        ret,sph_pos = vrep.simxGetObjectPosition(clientID,battery,-1,vrep.simx_opmode_streaming)
        ret,sens_pos = vrep.simxGetObjectPosition(clientID,sens,-1,vrep.simx_opmode_streaming)
        
        z = (sens_pos[2] - sph_pos[2])
        y = (sens_pos[1] - sph_pos[1])
        x = (sens_pos[0] - sph_pos[0])

        theta = float(-1*math.atan(x/(0.0001+z))*(180/3.14))
        theta_dot = (theta)*100.0  - (theta_init)*100.0
        theta_init = theta

        x_state = float(sens_pos[0])  
        x_state_dot = (x_state)*100.0 - (x_state_init)*100.0 
        x_state_init = x_state

        X[2][0] = theta
        X[3][0] = theta_dot
        X[0][0] = x_state
        X[1][0] = x_state_dot
        
        u = np.matmul(k,X) 
        
        print(theta_dot)
        speed_1 = -0.5 * u[0][0]
        speed_2 = -0.5 * u[1][0]

        #set torque here
        ret = vrep.simxSetJointTargetVelocity(clientID,motor1,speed_1,vrep.simx_opmode_oneshot)
        ret = vrep.simxSetJointTargetVelocity(clientID,motor2,speed_2,vrep.simx_opmode_oneshot)

if __name__ == "__main__":
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    if clientID!=-1:
        print ('Connected to remote API server')
        balance(clientID)
    else:
        print ('Failed connecting to remote API server')
        exit(0)
