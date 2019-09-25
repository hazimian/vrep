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

    def read_env(self,op_mode):
        returnCode ,cup_pos= vrep.simxGetObjectPosition(self.clientID, self.cup_handle, -1, op_mode)
        returnCode ,cup_ori= vrep.simxGetObjectPosition(self.clientID, self.cup_handle, -1, op_mode)
        returnCode ,ball_pos= vrep.simxGetObjectPosition(self.clientID, self.ball_handle, -1, op_mode)
        return np.reshape(cup_pos,[-1,1]),np.reshape(cup_ori,[-1,1]),np.reshape(ball_pos,[-1,1])

    def write_env(self,cup_pos,cup_ori,ball_pos,op_mode):
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.cup_handle, -1, cup_pos, op_mode)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.cup_handle, -1, cup_ori, op_mode)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ball_handle, -1, ball_pos, op_mode)

    def reset(self,q_home):
        #send robot to home
        self.Robot.SetArmJointTargetPos(q_home)
        #open gripper
        self.Robot.SetHandTargetVel([.1,.1,.1,.1])
        #randomly place the cup and the ball poses
        ball_pos = np.matmul(np.diag([.35, .25, 0]), np.random.random([3, 1])) + np.array([[0], [.2], [.22]])
        cup_pos = np.matmul(np.diag([.2, .25, 0]), np.random.random([3, 1])) + np.array([[-.35], [.2], [.24]])
        cup_ori=np.zeros([3,1])
        self.write_env(cup_pos,cup_ori,ball_pos,vrep.simx_opmode_blocking)
        self.Robot.SetHandTargetVel([0,0,0,0])


        #form state vector
        self.state=np.concatenate((q_home,q_g,cup_pos,cup_ori,ball_pos),axis=0)
        self.action=np.zeros(7,1)
        return self.state




    def step(self,action):
        dt=0.01
        qdot=action[:7,:]
        g_command=action[7:,:]

        #now apply the actions and observe the state and reward


    def reward(self):