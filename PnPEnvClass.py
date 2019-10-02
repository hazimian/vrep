from RobotClass import *

class PnP_Env(Robot):
    def __init__(self,clientID, FK_fun, IK_fun):
        self.clientID=clientID
        self.state=np.zeros([22,1])#state includes joint positions, gripper finger joint position, cup position, cup orientation, ball position,force
        self.action=np.zeros([10,1])#actions include joint velocities and gripper motor velocities
        _,self.cup_handle=vrep.simxGetObjectHandle(clientID, "Cup", vrep.simx_opmode_blocking)
        _,self.ball_handle=vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)

        #create a vrep robot object that is part of the environment
        super().__init__(clientID, FK_fun, IK_fun)

    def read_env(self,op_mode):
        returnCode ,cup_pos= vrep.simxGetObjectPosition(self.clientID, self.cup_handle, -1, op_mode)
        returnCode ,cup_ori= vrep.simxGetObjectOrientation(self.clientID, self.cup_handle, -1, op_mode)
        returnCode ,ball_pos= vrep.simxGetObjectPosition(self.clientID, self.ball_handle, -1, op_mode)
        dummy_pos=self.GetDummyPos()
        q=self.GetArmJointPos()
        q_g=self.GetHandJointPos()
        f=self.GetHandJointForce()
        state=np.concatenate((q,q_g,np.reshape(cup_pos,[-1,1]),np.reshape(cup_ori,[-1,1]),np.reshape(ball_pos,[-1,1]),f),axis=0)
        return state,dummy_pos

    def write_env(self,cup_pos,cup_ori,ball_pos,op_mode):
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.cup_handle, -1, cup_pos, op_mode)
        returnCode = vrep.simxSetObjectOrientation(self.clientID, self.cup_handle, -1, cup_ori, op_mode)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ball_handle, -1, ball_pos, op_mode)

    def reset(self,q_home):
        #randomly place the cup and the ball poses
        ball_pos = np.matmul(np.diag([.35, .25, 0]), np.random.random([3, 1])) + np.array([[0], [.2], [.22]])
        cup_pos = np.matmul(np.diag([.2, .25, 0]), np.random.random([3, 1])) + np.array([[-.35], [.2], [.24]])
        cup_ori=np.zeros([3,1])
        self.write_env(cup_pos,cup_ori,ball_pos,vrep.simx_opmode_oneshot)
        state, dummy_pos=self.read_env(vrep.simx_opmode_oneshot)#now read back the environment
        return state, dummy_pos





    def step(self,action):#runs one simulation step: apply actions and observe state
        qdot=action[:6]
        q_gdot=action[6:]
        self.SetHandTargetVel(q_gdot)
        self.SetArmJointTargetVel(qdot)
        vrep.simxSynchronousTrigger(self.clientID);  # Trigger next simulation step (Blocking function call)
        vrep.simxGetPingTime(self.clientID)  #a blocking call to ensure each step is actually executed
        state, dummy_pos=self.read_env(vrep.simx_opmode_oneshot)#now read back the environment
        r=self.reward(state)
        return state, dummy_pos,r

        #now apply the actions and observe the state and reward


    def reward(self,state):
        #distance between the cup and ball and cup orientation
        ball_pos=state[-7:-4]
        cup_pos=state[-13:-10]
        cup_ori=state[-10:-7]
        if np.linalg.norm(ball_pos-cup_pos)<.03 and np.linalg.norm(cup_ori[:2])<.1:
            r=1
        else:
            r=-1
        return r