import copy
from JacoKin import *
from PnPEnvClass import *
from expert_demo import demo_controller
from DDPGfDClass import *
import pickle
import matplotlib.pyplot as plt
from os import path


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
    if path.exists("data.pkl"):
        with open('data.pkl','rb') as f:
            buffer,episode0=pickle.load(f)
            episode0+=1
    else:
        episode0=0
        buffer=0

    #
    #     file_read_andle=open('data.pkl','r')
    #
    #
    # file_write_andle = open('data.obj', 'w')


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

        Env=PnP_Env(clientID, JacoFK, JacoIK)
        #get robot joint values and set this as home
        q = Env.GetArmJointPos()
        q_home=q


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
        N=4
        n_demos=2#numbe rof demos
        max_epoc=1000
        episode_length=int(8/t_s)
        batch_size=32
        #create a ddpg agent
        agent=ddpg(state_size=22,action_size=10,action_bound=np.array([3,3,3,3,3,3,.3,.3,.3,.3]),buffer=buffer)
        state_buffer = collections.deque(maxlen=agent.n)
        action_buffer = collections.deque(maxlen=agent.n)
        reward_buffer = collections.deque(maxlen=agent.n)
        for episode in range(episode0,max_epoc):
            # reset stage variable
            print("episode:", episode,":")
            stage = 0
            closing_time=0
            release_time=0
            total_return=0
            ###############Lets setup the scene##############
            vrep.simxSynchronous(clientID, True);  # Enable the synchronous mode(Blocking function call)
            returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
            # reset the environment
            a=np.zeros([22,1])
            state,dummy_pos=Env.reset(q_home)

            ####### ############step through simulation ########
            print("simulation")
            for step in range(episode_length):
                p1, R, _, _ = Env.FK_fun(q)
                p=dummy_pos

                if step % N == 0:
                    if episode < n_demos:
                    #generate action
                        a,stage,closing_time, release_time=demo_controller(state,p,R,stage, step,closing_time, release_time,t_s,N,Env,q_home)
                    else:
                        print(234)
                        a = agent.explore(np.squeeze(state))

                #apply action and simulate
                state_=state
                state,dummy_pos,r=Env.step(np.reshape(a,[-1,1]))#step through simulation and read back observations
                vrep.simxGetPingTime(clientID)
                transition = np.hstack((np.squeeze(state_), np.squeeze(a), r, np.squeeze(state)))
                total_return+=r
                # lets append this into buffers
                state_buffer.append(np.squeeze(state_))
                action_buffer.append(np.squeeze(a))
                reward_buffer.append(r)
                agent.record(transition, state_buffer, action_buffer, reward_buffer)  # insert it into replay buffer

            returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
            time.sleep(2)

            print("return:",total_return)
            #save the agent

            with open("data.pkl",'wb') as f:
                pickle.dump([agent.buffer,episode],f)
            #now train
            if episode>n_demos-1:
                print("training")
                for step in range(episode_length):
                    agent.train(batch_size, step)
        returnCode=vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):


        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')
