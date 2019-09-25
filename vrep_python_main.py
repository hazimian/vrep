import copy
from JacoKin import *
from RobotClass import *
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
        returnCode=vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)



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
            # send robot home
            returnCode = Robot1.SetArmJointTargetPos(q_home)
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
                                                                       vrep.simx_opmode_blocking)
                returnCode, Jaco_Hand_ori = vrep.simxGetObjectOrientation(clientID, HandCenterDummy_handle, -1,
                                                                          vrep.simx_opmode_blocking)
                p = np.reshape(Jaco_Hand_pos, [-1, 1])
                # R = Euler2Rot(Jaco_Hand_pos)

                q = Robot1.GetArmJointPos()
                p1, R, _, _ = Robot1.FK_fun(q)
                #wait for 2 seconds
                if step==int(1/t_s):
                    ########## Now get the scene##############
                    returnCode, sphere_pos = vrep.simxGetObjectPosition(clientID, sphere_handle, -1,
                                                                        vrep.simx_opmode_blocking)
                    pd = np.reshape(sphere_pos, [-1, 1])+np.array([[0], [0], [.08]])

                    stage=1
                    print('stage:',stage)
                    continue

                if stage==1 and step%N==0:
                    ###move towards the ball
                    ep = pd-p
                    eo = np.reshape(.5*np.cross(np.ndarray.flatten(R[:,2]),np.array([0,0,-1])),[-1,1])

                    if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
                        qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q,p,R, 100, 100)
                        Robot1.SetArmJointTargetPos(qd)
                        #Robot1.SetArmJointTargetPos(qdot)

                    else:
                        stage=2
                        print('stage:', stage)
                        continue
                    # self.SetArmJointTargetPos(qdot)

                if stage == 2 and step%N==0:# now lets descend and grasp
                    pd = np.reshape(sphere_pos, [-1, 1])+np.array([[0], [0], [.01]])
                    ###move towards the ball
                    ep = pd-p
                    eo = np.reshape(.5*np.cross(np.ndarray.flatten(R[:,2]),np.array([0,0,-1])),[-1,1])
                    if np.linalg.norm(ep) > .003 or np.linalg.norm(eo) > .01:
                        qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q,p,R, 10, 10)
                        Robot1.SetArmJointTargetPos(qd)
                    else:
                        stage=3
                        print('stage:', stage)

                        continue

                if stage == 3 and step % N == 0: # closing the grasp
                    closing_time=closing_time+1
                    if closing_time<int(2/(N*t_s)):
                        Robot1.SetHandTargetVel([-.05, -.05, -.05, -.05])
                    else:
                        stage=4
                        print('stage:', stage)
                        continue
                if stage == 4 and step % N == 0:  #now move above the cup

                    pd=cup_pos+np.array([[0], [0], [.2]])
                    ep = pd - p
                    eo = np.reshape(.5 * np.cross(np.ndarray.flatten(R[:, 2]), np.array([0, 0, -1])), [-1, 1])
                    if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
                        qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q,p,R, 100, 100)
                        Robot1.SetArmJointTargetPos(qd)
                    else:
                        stage = 5
                        print('stage:', stage)
                        continue

                if stage == 5 and step % N == 0:  #descend

                    pd=cup_pos+np.array([[0], [0], [.1]])
                    ep = pd - p
                    eo = np.reshape(.5 * np.cross(np.ndarray.flatten(R[:, 2]), np.array([0, 0, -1])), [-1, 1])
                    if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
                        qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q,p,R, 10, 10)
                        Robot1.SetArmJointTargetPos(qd)
                    else:
                        stage = 6
                        print('stage:', stage)
                        continue

                if stage == 6 and step % N == 0:  # now lets descend towards the cup and release
                    release_time=release_time+1
                    if release_time<int(1/(N*t_s)):
                        Robot1.SetHandTargetVel([.2, .2, .2, .2])
                    else:
                        stage=7
                        print('stage:', stage)
                        break

                    # #let move robot back to home and ready for next run
                    # Robot1.SetArmJointTargetPos(q_home)
                    # #wait until roobt is home
                    # while np.linalg.norm(q-q_home)>.01:
                    #     time.sleep(.01)
                    #     q=Robot1.GetArmJointPos()
        returnCode=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

     # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')
