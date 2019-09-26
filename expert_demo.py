import numpy as np

def demo_controller(p,R,q,sphere_pos,cup_pos,stage,step,closing_time, release_time,t_s,N,Robot1,q_home):
    u=np.zeros([10,1])
    if step == int(1 / t_s):#idle: do nothing just send zero control effort
        stage = 1
        print('stage:', stage)
        return u,stage,closing_time, release_time

    if stage == 1:
        ###move towards the ball
        pd = np.reshape(sphere_pos, [-1, 1]) + np.array([[0], [0], [.08]])
        ep = pd - p
        eo = np.reshape(.5 * np.cross(np.ndarray.flatten(R[:, 2]), np.array([0, 0, -1])), [-1, 1])

        if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
            qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q, p, R, 10, 10)
            q_gdot=np.zeros([4,1])
            u=np.concatenate((qdot,q_gdot),axis=0)
            return u,stage,closing_time, release_time

        else:
            stage = 2
            print('stage:', stage)
            # return u,stage,closing_time, release_time

    if stage == 2 :  # now lets descend and grasp
        pd = np.reshape(sphere_pos, [-1, 1]) + np.array([[0], [0], [.01]])
        ###move towards the ball
        ep = pd - p
        eo = np.reshape(.5 * np.cross(np.ndarray.flatten(R[:, 2]), np.array([0, 0, -1])), [-1, 1])
        if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
            qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q, p, R, 10, 10)
            q_gdot=np.zeros([4,1])
            u=np.concatenate((qdot,q_gdot),axis=0)
            return u,stage,closing_time, release_time
        else:
            stage = 3
            print('stage:', stage)
            # return u,stage,closing_time, release_time


    if stage == 3 :  # closing the grasp
        closing_time = closing_time + 1
        if closing_time < int(3 / (N * t_s)):
            #Robot1.SetHandTargetVel([-.1, -.1, -.1, -.1])
            qdot=np.zeros([6,1])
            q_gdot=np.array([[-.3],[-.2],[-.3],[-.2]])
            u=np.concatenate((qdot,q_gdot),axis=0)
            return u,stage,closing_time, release_time
        else:
            stage = 4
            print('stage:', stage)
            # return u,stage,closing_time, release_time
    if stage == 4:  # now move above the cup

        pd = np.reshape(cup_pos, [-1, 1])  + np.array([[0], [0], [.2]])
        ep = pd - p
        eo = np.reshape(.5 * np.cross(np.ndarray.flatten(R[:, 2]), np.array([0, 0, -1])), [-1, 1])
        if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
            qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q, p, R, 10, 10)
            q_gdot=-.2*np.ones([4,1])
            u=np.concatenate((qdot,q_gdot),axis=0)
            return u,stage,closing_time, release_time
        else:
            stage = 5
            print('stage:', stage)
            # return u,stage,closing_time, release_time

    if stage == 5:  # descend

        pd = np.reshape(cup_pos, [-1, 1])  + np.array([[0], [0], [.1]])
        ep = pd - p
        eo = np.reshape(.5 * np.cross(np.ndarray.flatten(R[:, 2]), np.array([0, 0, -1])), [-1, 1])
        if np.linalg.norm(ep) > .001 or np.linalg.norm(eo) > .01:
            qd, p, ep, eo, qdot = Robot1.IK_fun(pd, q, p, R, 10, 10)
            q_gdot=-.2*np.ones([4,1])
            u=np.concatenate((qdot,q_gdot),axis=0)
            return u,stage,closing_time, release_time
        else:
            stage = 6
            print('stage:', stage)
            # return u,stage,closing_time, release_time

    if stage == 6:  # now release
        release_time = release_time + 1
        if release_time < int(1 / (N * t_s)):
            qdot=np.zeros([6,1])
            q_gdot=.2*np.ones([4,1])
            u=np.concatenate((qdot,q_gdot),axis=0)
            return u,stage,closing_time, release_time
        else:
            stage = 7
            print('stage:', stage)
            # return u,stage,closing_time, release_time
    # if stage == 7 and step % N == 0:  # now go back home
    #
    #
    #     eq = q_home - q
    #     if np.linalg.norm(eq) > .01:
    #         qdot=10*eq
    #         q_gdot=np.zeros([4,1])
    #         u=np.concatenate((qdot,q_gdot),axis=0)
    #         return u,stage,closing_time, release_time
    #     else:
    #         stage = 8
    #         print('stage:', stage)
    #         return u,stage,closing_time, release_time
    return u,stage,closing_time, release_time
