##Created by Hamidreza Azimian
## Epson Canada
## September 2019
import numpy as np
import copy
import tensorflow as tf
import collections
from functools import partial

my_dense_layer = partial(
    tf.layers.dense, kernel_regularizer=tf.contrib.layers.l2_regularizer(.01))

class ddpg:
    def __init__(self,state_size,action_size,action_bound,buffer):
        self.gamma=.99
        self.lr=.001

        # n-step return
        self.n = 5
        self.state_size=state_size
        self.action_size=action_size
        self.action_bound=action_bound

        self.state=tf.placeholder(dtype=tf.float32,shape=[None,state_size],name='s_')
        self.reward = tf.placeholder(dtype=tf.float32, shape=[None, 1],name='r')
        self.next_state = tf.placeholder(dtype=tf.float32, shape=[None, state_size],name='s')
        self.g = tf.placeholder(dtype=tf.float32, shape=[None, 1],name='gamma')# this can be gamma or gamma^n
        self.l = tf.placeholder(dtype=tf.float32, shape=[None, 1],name='lambda')# this can be gamma or gamma^n
        ######c#reate replay buffer######
        if buffer:
            self.buffer=buffer
        else:
            self.buffer=collections.deque(maxlen=1000000)

        ######## build actor network#####
        with tf.variable_scope('Actor'):
            self.a=self.build_actor(self.state,'eval')
            self.a_tar = self.build_actor(self.next_state,'target')

        ######### build critic which has special architecture######
        with tf.variable_scope('Critic'):
            self.Q=self.build_critic(self.a,self.state,'eval')
            self.Q_tar=self.build_critic(self.a_tar,self.next_state,'target')
            # initialize the network parameters
        self.ae_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval')
        self.at_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target')

        self.ce_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval')
        self.ct_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target')

        alpha = .01
        # update target value
        self.soft_replace = [
            [tf.assign(at, (1 - alpha) * at + alpha * ae), tf.assign(ct, (1 - alpha) * ct + alpha * ce)]
            for at, ae, ct, ce in zip(self.at_params, self.ae_params, self.ct_params, self.ce_params)]

        # create critic trainign node
        c_reg_losses = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES,scope='Critic/eval')
        c_base_loss=tf.losses.mean_squared_error(labels=self.l*(self.reward + self.g * self.Q_tar), predictions=self.l*self.Q)
        self.c_loss = tf.add_n([c_base_loss] + c_reg_losses, name="loss")
        c_optimizer = tf.train.AdamOptimizer(self.lr)
        self.c_train_op = c_optimizer.minimize(self.c_loss, var_list=self.ce_params)

        # create actor training node
        a_reg_losses = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES,scope='Actor/eval')#get regularization term
        a_base_loss=-tf.reduce_mean(self.Q)
        self.a_loss=tf.add_n([a_base_loss] + a_reg_losses, name="loss")
        a_optimizer=tf.train.AdamOptimizer(self.lr)
        self.a_train_op=a_optimizer.minimize(self.a_loss, var_list=self.ae_params)
        self.sess=tf.Session()






        ##initialize
        self.sess.run(tf.global_variables_initializer())

    def record(self,transition,state_buffer,action_buffer,reward_buffer):
        #insert gamma
        transition=np.append(transition,self.gamma)
        self.buffer.appendleft(transition)
        # also record n-step transition
        if len(state_buffer)==self.n:
            #initialize the n-step transition
            n_transition=copy.deepcopy(transition)
            #insert state and action of n steps ago
            n_transition[:self.state_size]=state_buffer[0]
            n_transition[self.state_size:self.state_size+self.action_size]=action_buffer[0]
            # calculate n-step return
            ret_n=0
            for i in range(self.n):
                ret_n=ret_n+np.power(self.gamma,i)*reward_buffer[i]
            n_transition[ self.state_size + self.action_size ] = ret_n
            n_transition[2*self.state_size+self.action_size+1]=np.power(self.gamma,self.n)
            self.buffer.appendleft(n_transition)


        #n_transition=self.buffer.appendleft(transition+(np.power(self.gamma,self.n),))

    def train(self,batch_size,i):
        self.sess.run(self.soft_replace)
        inds=np.random.choice(len(self.buffer),batch_size)
        state_batch=np.zeros([batch_size,self.state_size])
        action_batch = np.zeros([batch_size, self.action_size])
        reward_batch = np.zeros([batch_size, 1])
        next_state_batch = np.zeros([batch_size, self.state_size])
        g_batch=np.zeros([batch_size,1])
        l_batch=np.zeros([batch_size,1])
        for i in range(batch_size):
            state_batch[i,:]=self.buffer[inds[i]][:self.state_size]
            action_batch[i, :] = self.buffer[inds[i]][self.state_size:self.state_size+self.action_size]
            reward_batch[i, :] = self.buffer[inds[i]][self.state_size+self.action_size:self.state_size+self.action_size+1]
            next_state_batch[i, :] = self.buffer[inds[i]][self.state_size + self.action_size + 1:2*self.state_size + self.action_size+1]
            g_batch[i,:]=self.buffer[inds[i]][2*self.state_size + self.action_size+1]
            if g_batch[i,:]<self.gamma:
                l_batch[i, :] = 0.4
            else:
                l_batch[i, :] = 1
        if i%1==0:
            self.sess.run(self.c_train_op,{self.state:state_batch,self.a:action_batch,self.reward:reward_batch,self.next_state:next_state_batch,self.g:g_batch,self.l:l_batch})
            self.sess.run(self.a_train_op,{self.state:state_batch})


    def explore(self,s):
        #noise = self.OrnsteinUhlenbeckActionNoise(mu=np.zeros(self.action_size))
        noise=np.random.multivariate_normal(np.zeros(self.action_size),.1*np.identity(self.action_size),1)
        a=self.sess.run(self.a,{self.state:s[np.newaxis,:]})[0]
        return np.clip(a+noise[0],-self.action_bound,self.action_bound)
    def build_actor(self,state,scope):

        with tf.variable_scope(scope):
            a_l1 = my_dense_layer(inputs=state, units=50, activation=tf.nn.relu, trainable=True)
            a_l2 = my_dense_layer(inputs=a_l1, units=50, activation=tf.nn.relu, trainable=True)
            return tf.multiply(tf.layers.dense(inputs=a_l2, units=self.action_size, activation=tf.nn.tanh, trainable=True, name='a'),
                                 self.action_bound, name='scaled_a')

    def build_critic(self, action,state,scope):
        with tf.variable_scope(scope):
            Q_l1 = my_dense_layer(inputs=action, units=100, activation=tf.nn.relu, trainable=True)
            Q_l2 = my_dense_layer(inputs=tf.concat(axis=1, values=[state, Q_l1]), units=100, activation=tf.nn.relu,
                                   trainable=True)
            return my_dense_layer(inputs=Q_l2, units=1, trainable=True)

    class OrnsteinUhlenbeckActionNoise:
        def __init__(self, mu, sigma=.3, theta=.15, dt=1e-2, x0=None):
            self.theta = theta
            self.mu = mu
            self.sigma = sigma
            self.dt = dt
            self.x0 = x0
            self.reset()

        def __call__(self):
            x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + \
                self.sigma * np.sqrt(self.dt) * np.random.normal(size=self.mu.shape)
            self.x_prev = x
            return x

        def reset(self):
            self.x_prev = self.x0 if self.x0 is not None else np.zeros_like(self.mu)

        def __repr__(self):
            return 'OrnsteinUhlenbeckActionNoise(mu={}, sigma={})'.format(self.mu, self.sigma)
