import time
import numpy as np
import vrep
class Robot:
    def __init__(self, clientID, FK_fun, IK_fun):
        self.clientID = clientID
        self.FK_fun = FK_fun
        self.IK_fun = IK_fun
        self.arm_joints_handle = self.GetArmHandles()
        self.hand_joints_handle = self.GetHandHandles()


    def GetArmHandles(self):
        arm_joints_handle = np.zeros(6)
        for i in range(0, 6):
            returnCode, arm_joints_handle[i] = vrep.simxGetObjectHandle(self.clientID, "Jaco_joint" + str(i + 1),
                                                                        vrep.simx_opmode_blocking)
            returnCode = vrep.simxGetJointPosition(self.clientID, int(arm_joints_handle[i]), vrep.simx_opmode_streaming)
            time.sleep(.5)

        return arm_joints_handle

    def GetHandHandles(self):
        hand_joints_handle = np.zeros(4)
        returnCode, F3_M1_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_finger3_motor1",
                                                            vrep.simx_opmode_blocking)
        returnCode, F3_M2_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_finger3_motor2",
                                                            vrep.simx_opmode_blocking)
        returnCode, F12_M1_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_fingers12_motor1",
                                                             vrep.simx_opmode_blocking)
        returnCode, F12_M2_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_fingers12_motor2",
                                                             vrep.simx_opmode_blocking)

        returnCode, F1_J2_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_joint2_finger1",
                                                            vrep.simx_opmode_blocking)
        returnCode, F2_J2_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_joint2_finger2",
                                                             vrep.simx_opmode_blocking)
        returnCode, F3_J2_handle = vrep.simxGetObjectHandle(self.clientID, "JacoHand_joint2_finger3",
                                                             vrep.simx_opmode_blocking)

        hand_joints_handle = [F3_M1_handle, F3_M2_handle, F12_M1_handle, F12_M2_handle,F1_J2_handle,F2_J2_handle,F3_J2_handle]
        return hand_joints_handle


    def IK_Contol(self,pd, q,ep_th,eo_th,kp,ko):
        ep = 100
        eo = 100
        while ep > ep_th or eo > eo_th:
            qd, p, ep, eo, qdot = self.IK_fun(pd, q,kp,ko)
            self.SetArmJointTargetPos(qd)
            #self.SetArmJointTargetPos(qdot)
            time.sleep(.01)
            q = self.GetArmJointPos()
        return q

    def GetArmJointPos(self):
        q=np.zeros([6,1])
        for i in range(0, 6):
            returnCode, q[i,0] = vrep.simxGetJointPosition(self.clientID, int(self.arm_joints_handle[i]), vrep.simx_opmode_buffer)
        return q

    def SetArmJointTargetPos(self,qd):
        for i in range(0, 6):
            returnCode= vrep.simxSetJointTargetPosition(self.clientID, int(self.arm_joints_handle[i]), qd[i,0],vrep.simx_opmode_streaming)
        return returnCode


    def SetHandTargetVel(self,vel_d):
        for i in range(0,4):
            returnCode = vrep.simxSetJointTargetVelocity(self.clientID, int(self.hand_joints_handle[i]), vel_d[i], vrep.simx_opmode_oneshot)

        return returnCode


        return returnCode
    def GetHandJointPos(self):
        q=np.zeros([3,1])
        for i in range(3):
            returnCode,q[i,0] = vrep.simxGetJointPosition(self.clientID, int(self.hand_joints_handle[i+4]), vrep.simx_opmode_blocking)
        return q

    def SetArmJointTargetVel(self,qdot):
        for k in range(6):
            returnCode=vrep.simxSetJointTargetVelocity(self.clientID, int(self.arm_joints_handle[k]), qdot[k, 0],
                                            vrep.simx_opmode_oneshot)
        return returnCode

