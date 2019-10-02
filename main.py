import gym
from DDPGfDClass import *





#testing git
if __name__=="__main__":
    batch_size=32
    Env = gym.make("Pendulum-v0")
    Env = Env.unwrapped
    Env.seed(1)

    action_bound=Env.action_space.high
    state_size = Env.observation_space.shape[0]
    action_size = Env.action_space.shape[0]
    a_bound = Env.action_space.high
    agent=ddpg(state_size,action_size,action_bound)
    state_buffer=collections.deque(maxlen=agent.n)
    action_buffer=collections.deque(maxlen=agent.n)
    reward_buffer=collections.deque(maxlen=agent.n)
    epoch_max=1000
    episode_length=400
    learning_steps=1
    for epoch in range(epoch_max):
        s = Env.reset()
        a_vec = []
        total_reward=0
        for i in range(episode_length):
            a=agent.explore(s)
            a_vec.append(a)
            s_=s
            s,r,_,_= Env.step(a)
            total_reward+=r
            Env.render()
            transition=np.hstack((s_,a,r,s))
            #lets append this into buffers
            state_buffer.append(s_)
            action_buffer.append(a)
            reward_buffer.append(r)
            agent.record(transition,state_buffer,action_buffer,reward_buffer)#insert it into replay buffer


        for i in range(episode_length*learning_steps):
            agent.train(batch_size,i)
        print('Epoch:',epoch,' ','total reward:', total_reward)
        print(max(a_vec))