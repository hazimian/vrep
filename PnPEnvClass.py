from RobotClass import *

class PnP_Env(Robot):
    def __init__(self,clientID, FK_fun, IK_fun):
        self.clientID=clientID
        self.state=np.zeros(18,1)#state includes joint positions, gripper finger joint position, cub position, cup orientation, ball position
        self.action=np.zeros([10,1])#actions include joint velocities and gripper motor velocities
        self.cup_handle=vrep.simxGetObjectHandle(clientID, "Cup", vrep.simx_opmode_blocking)
        self.ball_handle=vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)

        #create a vrep robot object that is part of the environment
        super().__init__(clientID, FK_fun, IK_fun)

    def read_env(self):
        returnCode ,cup_pos= vrep.simxGetObjectPosition(self.clientID, self.cup_handle, -1, vrep.simx_opmode_blocking)
        returnCode ,cup_ori= vrep.simxGetObjectPosition(self.clientID, self.cup_handle, -1, vrep.simx_opmode_blocking)
        returnCode ,ball_pos= vrep.simxGetObjectPosition(self.clientID, self.ball_handle, -1, vrep.simx_opmode_blocking)
        return np.reshape(cup_pos,[-1,1]),np.reshape(cup_ori,[-1,1]),np.reshape(ball_pos,[-1,1])

    def write_env(self,cup_pos,cup_ori,ball_pos):
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.cup_handle, -1, cup_pos, vrep.simx_opmode_blocking)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.cup_handle, -1, cup_ori, vrep.simx_opmode_blocking)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ball_handle, -1, ball_pos, vrep.simx_opmode_blocking)

    def reset(self,ball_pos,cup_pos,cup_ori,q_home):
        #send robot to home
        self.Robot.SetArmJointTargetPos(q_home)
        #open gripper
        self.Robot.SetHandTargetVel([.1,.1,.1,.1])
        #set object poses
        self.write_env(cup_pos,cup_ori,ball_pos)
        #now let's wait and then read the actual objects poses
        time.sleep(3)
        self.Robot.SetHandTargetVel([0,0,0,0])
        q=self.Robot.GetArmJointPos()
        q_g=self.Robot.GetHandJointPos()
        cup_pos,cup_ori,ball_pos=self.read_env()

        #form state vector
        self.state=np.concatenate((q,q_g,cup_pos,cup_ori,ball_pos),axis=0)
        self.action=np.zeros(7,1)
        return self.state




    def step(self,action):
        dt=0.01
        qdot=action[:7,:]
        g_command=action[7:,:]

        #now apply the actions and observe the state and reward


    def reward(self):