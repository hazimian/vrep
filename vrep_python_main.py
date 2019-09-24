import copy
from JacoKin import *
from RobotClass import *
import matplotlib.pyplot as plt






if __name__=='__main__':
    # try:
    #     import vrep
    #
    # except:
    #     print ('--------------------------------------------------------------')
    #     print ('"vrep.py" could not be imported. This means very probably that')
    #     print ('either "vrep.py" or the remoteApi library could not be found.')
    #     print ('Make sure both are in the same folder as this file,')
    #     print ('or appropriately adjust the file "vrep.py"')
    #     print ('--------------------------------------------------------------')
    #     print ('')


    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    if clientID!=-1:
        print ('Connected to remote API server')
        # create robot object
        Robot1 = Robot(clientID, JacoFK, JacoIK)
        ##get handles
        returnCode, sphere_handle = vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)
        #get cup handle
        returnCode, cup_handle = vrep.simxGetObjectHandle(clientID, "Cup", vrep.simx_opmode_blocking)
        ##get handle to Jaco hand dummy
        returnCode,HandCenterDummy_handle=vrep.simxGetObjectHandle(clientID, "HandCenterDummy", vrep.simx_opmode_blocking)

        # Robot1.SetHandTargetVel([-.1, -.1, -.1, -.1])
        # time.sleep(1)
        # returnCode, x = vrep.simxGetJointPosition(clientID, int(Robot1.hand_joints_handle[0]),
        #                                                 vrep.simx_opmode_buffer)

        #get robot joint values and set this as home
        q = Robot1.GetArmJointPos()
        q_home=q
        returnCode, Jaco_Hand_pos = vrep.simxGetObjectPosition(clientID, HandCenterDummy_handle, -1, vrep.simx_opmode_blocking)
        returnCode, cup_ori = vrep.simxGetObjectOrientation(clientID, cup_handle, -1, vrep.simx_opmode_blocking)



        for iter in range(10):
            #randomly place the sphere and the cup
            sphere_pos=np.matmul(np.diag([.35,.25,0]),np.random.random([3,1]))+np.array([[0],[.2],[.22]])
            returnCode=vrep.simxSetObjectPosition(clientID,sphere_handle,-1,sphere_pos,vrep.simx_opmode_blocking)
            cup_pos=np.matmul(np.diag([.2,.25,0]),np.random.random([3,1]))+np.array([[-.35],[.2],[.24]])
            returnCode=vrep.simxSetObjectPosition(clientID,cup_handle,-1,cup_pos,vrep.simx_opmode_blocking)
            returnCode = vrep.simxSetObjectOrientation(clientID, cup_handle, -1,cup_ori,vrep.simx_opmode_blocking)

            time.sleep(1)
            #get teh actual positions and set sphere position as the target
            returnCode, sphere_pos = vrep.simxGetObjectPosition(clientID, sphere_handle, -1,vrep.simx_opmode_blocking)
            pd=np.reshape(sphere_pos,[-1,1])

            #get the current hand position
            returnCode, Jaco_Hand_pos = vrep.simxGetObjectPosition(clientID, HandCenterDummy_handle, -1,vrep.simx_opmode_blocking)
            p = np.reshape(Jaco_Hand_pos, [-1, 1])
            #IK loop
            q=Robot1.IK_Contol(pd + np.array([[0], [0], [.09]]), q,.001,.01,100,100)
            #now lets descend and grasp
            q=Robot1.IK_Contol(pd + np.array([[0], [0], [.01]]), q,.003,.1,10,10)
            #closing the grasp
            Robot1.SetHandTargetVel([-.1,-.1,-.1,-.1])
            time.sleep(3)

            #now move above the cup
            pd=cup_pos
            q = Robot1.IK_Contol(pd + np.array([[0], [0], [.2]]), q, .001, .01, 100, 100)

            #descend
            q = Robot1.IK_Contol(pd + np.array([[0], [0], [.12]]), q, .001, .01, 10, 10)
            Robot1.SetHandTargetVel([.1, .1, .1, .1])
            time.sleep(1)

            #let move robot back to home and ready for next run
            Robot1.SetArmJointTargetPos(q_home)
            #wait until roobt is home
            while np.linalg.norm(q-q_home)>.01:
                time.sleep(.01)
                q=Robot1.GetArmJointPos()

     # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')
