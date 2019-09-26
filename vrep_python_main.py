import copy
from JacoKin import *
from PnPEnvClass import *
from expert_demo import demo_controller
import matplotlib.pyplot as plt

def Euler2Rot(angles):
    a=angles[0]
    b=angles[1]
    g=angles[2]
    # R=np.array([[cos(b)*cos(g), -cos(b)*sin(g), sin(b)],
    #             [cos(a)*sin(g)+sin(a)*sin(b)*cos(g), cos(a)*cos(g)-sin(a)*sin(b)*sin(g), -sin(a)*cos(b)],
    #             [sin(a)*sin(g)-cos(a)*sin(b)*cos(g), sin(a)*cos(g)+cos(a)*sin(b)*sin(g), cos(a)*cos(b)]])

    R1=np.array([[1,0,0],[0,cos(a),-sin(a)],[0,sin(a),cos(a)]])
    R2=np.array([[cos(b),0,sin(b)],[0,1,0],[-sin(b),0,cos(b)]])
    R3=np.array([[cos(g),-sin(g),0],[sin(g),cos(g),0],[0,0,1]])

    R=np.matmul(np.matmul(R1,R2),R3)

    return R





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
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP (19999 for non-continuous mode and 19997 for continuous)
    if clientID!=-1:
        print ('Connected to remote API server')
        vrep.simxSynchronous(clientID, True); # Enable the synchronous mode(Blocking function call)




        # create robot object
        Robot1 = Robot(clientID, JacoFK, JacoIK)
        ##get handles
        returnCode, sphere_handle = vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)
        #get cup handle
        returnCode, cup_handle = vrep.simxGetObjectHandle(clientID, "Cup", vrep.simx_opmode_blocking)
        ##get handle to Jaco hand dummy
        returnCode,HandCenterDummy_handle=vrep.simxGetObjectHandle(clientID, "HandCenterDummy", vrep.simx_opmode_blocking)


        #get robot joint values and set this as home
        q = Robot1.GetArmJointPos()
        q_home=q
        returnCode, Jaco_Hand_pos = vrep.simxGetObjectPosition(clientID, HandCenterDummy_handle, -1, vrep.simx_opmode_blocking)
        returnCode, cup_ori = vrep.simxGetObjectOrientation(clientID, cup_handle, -1, vrep.simx_opmode_blocking)

        #task squence:
        # 0. sit idle for scene initial setup (move robot to home, randomly place the ball and the cup)
        # 1. approach above the ball
        # 2. descend towards the ball
        # 3. grasp the ball
        # 4. approach above the cup
        # 5. descend towards the ball
        # 6. release
        # 7. done

        t_s=.01
        N=2

        for episode in range(10):
            # reset stage variable
            print("##### Episode", episode,":")
            stage = 0
            closing_time=0
            release_time=0
            ###############Lets setup the scene##############
            returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
            # randomly place the sphere and the cup
            sphere_pos = np.matmul(np.diag([.35, .25, 0]), np.random.random([3, 1])) + np.array([[0], [.2], [.22]])
            returnCode = vrep.simxSetObjectPosition(clientID, sphere_handle, -1, sphere_pos, vrep.simx_opmode_blocking)
            cup_pos = np.matmul(np.diag([.2, .25, 0]), np.random.random([3, 1])) + np.array([[-.35], [.2], [.24]])
            returnCode = vrep.simxSetObjectPosition(clientID, cup_handle, -1, cup_pos, vrep.simx_opmode_blocking)
            returnCode = vrep.simxSetObjectOrientation(clientID, cup_handle, -1, cup_ori, vrep.simx_opmode_blocking)

            ####### ############step through simulation ########
            for step in range(int(50/t_s)):
                vrep.simxSynchronousTrigger(clientID);  # Trigger next simulation step (Blocking function call)
                # vrep.simxGetPingTime(clientID)  #a blocking call to ensure each step is actually executed

                # get the current hand position
                returnCode, Jaco_Hand_pos = vrep.simxGetObjectPosition(clientID, HandCenterDummy_handle, -1,
                                                                       vrep.simx_opmode_oneshot)
                returnCode, Jaco_Hand_ori = vrep.simxGetObjectOrientation(clientID, HandCenterDummy_handle, -1,
                                                                          vrep.simx_opmode_oneshot)
                p = np.reshape(Jaco_Hand_pos, [-1, 1])
                # R = Euler2Rot(Jaco_Hand_pos)
                q = Robot1.GetArmJointPos()
                p1, R, _, _ = Robot1.FK_fun(q)
                returnCode, sphere_pos = vrep.simxGetObjectPosition(clientID, sphere_handle, -1,
                                                                          vrep.simx_opmode_oneshot)
                returnCode, cup_pos = vrep.simxGetObjectPosition(clientID, cup_handle, -1,
                                                                          vrep.simx_opmode_oneshot)
                q = Robot1.GetArmJointPos()


                if step%N==0:
                    u,stage,closing_time, release_time=demo_controller(p, R, q, sphere_pos, cup_pos, stage, step,  closing_time, release_time,t_s,N,Robot1,q_home)


                qdot=np.zeros([6,1])
                qdot=u[0:6]
                Robot1.SetArmJointTargetVel(qdot)
                q_gdot=u[6:]
                Robot1.SetHandTargetVel(q_gdot)
                if stage==7:
                    returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
                    time.sleep(2)
                    break

        returnCode=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)


     # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')
