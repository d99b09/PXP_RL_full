import vrep
from coord_to_polar import *
import sys
import random as r
import math as m
from time import sleep
import numpy as np
import itertools
import time



class PXP_env():
    OBSERVATION_SPACE_VALUES = (17,)
    ACTION_SPACE_SIZE = 81
    

    def __init__(self):

        self.clientID = None
        self.handle_joint_1 = None
        self.handle_joint_2 = None
        self.handle_joint_3 = None
        self.handle_joint_4 = None
        self.handle_end_pincher = None
        self.handle_ball = None
        self.handle_end_tester = None
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0
        self.ball_coord = None
        self.ball_coord_x = None
        self.ball_coord_y = None
        self.ball_coord_z = None
        self.end_coord = None
        self.end_coord_x = None
        self.end_coord_y = None
        self.end_coord_z = None
        self.obs = None
        self.dist = None
        self.num_steps = 0
        self.render_steps = 0
        self.configurate()
        self.action_choice = self.choice()







    def step(self, action):

        self.distance()
        start_dist = self.dist

        action_set = self.action_choice[action]

        if action_set[0] == -1:
            self.q1 -= m.radians(2)

        elif action_set[0] == 1:
            self.q1 += m.radians(2)

        if action_set[1] == -1:
            self.q2 -= m.radians(2)

        elif action_set[1] == 1:
            self.q2 += m.radians(2)

        if action_set[2] == -1:
            self.q3 -= m.radians(2)

        elif action_set[2] == 1:
            self.q3 += m.radians(2)

        if action_set[3] == -1:
            self.q4 -= m.radians(2)

        elif action_set[3] == 1:
            self.q4 += m.radians(2)

        self.setJoint_PXP()
        self.get_end_coord()
        obs = self.get_obs()
        d_dist = start_dist - self.dist
        reward = (d_dist * 25) ** 2

        done = False

        self.num_steps += 1

        if self.dist < 0.05:
            reward += 1000*0.9**self.num_steps
            done = True
        elif self.num_steps > 300:
            done = True





        return obs, reward, done








    def reset(self):

        self.num_steps = 0

        self.set_randomJoint_PXP()

        self.create_random_ball()

        return self.get_obs()






        #raise NotImplementedError()

    def configurate(self):
        print('Program started')

        vrep.simxFinish(-1)

        self.clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.clientID != -1:
            print("Connected to remote server")
        else:
            print('Connection not successful')
            sys.exit('Could not connect')
        errorCode, self.handle_joint_1 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint1',
                                                                  vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_joint_2 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint2',
                                                                  vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_joint_3 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint3',
                                                                  vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_joint_4 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint4',
                                                                  vrep.simx_opmode_oneshot_wait)

        errorCode, self.handle_end_pincher = vrep.simxGetObjectHandle(self.clientID,
                                                                      'PhantomXPincher_gripperCenter_joint',
                                                                      vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_ball = vrep.simxGetObjectHandle(self.clientID,
                                                               'Dummy',
                                                               vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_end_tester = vrep.simxGetObjectHandle(self.clientID,
                                                               'Dummy0',
                                                               vrep.simx_opmode_oneshot_wait)

        print('Configuration ended')

    def render(self, mode='human', close=False):

        self.render_steps += 1

        print('num_render:', self.render_steps)

        #raise NotImplementedError()

    def close(self):

        print('close me')

        #raise NotImplementedError()


    def setJoint_PXP(self):
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_1, self.q1, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_2, self.q2, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_3, self.q3, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_4, self.q4, vrep.simx_opmode_oneshot_wait)


    def set_randomJoint_PXP(self):
        self.q1 = m.radians(r.randrange(-180, 180, 2))
        self.q2 = m.radians(r.randrange(-90, 90, 2))
        self.q3 = m.radians(r.randrange(-90, 90, 2))
        self.q4 = m.radians(r.randrange(0, 90, 2))
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_1, self.q1, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_2, self.q2, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_3, self.q3, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_4, self.q4, vrep.simx_opmode_oneshot_wait)
        self.get_end_coord()




    def create_random_ball(self):
        ro = r.uniform(0.1357, 0.31061)
        alf = r.uniform(0, 2 * m.pi)
        fi = r.uniform(0, 2 * m.pi)

        self.ball_coord_z = 1.2892e-01 + ro * m.sin(alf)

        while self.ball_coord_z < 0:
            ro = r.uniform(0.1357, 0.31061)
            alf = r.uniform(0, 2 * m.pi)
            fi = r.uniform(0, 2 * m.pi)
            self.ball_coord_z = 1.2892e-01 + ro * m.sin(alf)

        self.ball_coord_x = ro * m.cos(alf) * m.cos(fi)
        self.ball_coord_y = -3.9390e-02 + ro * m.cos(alf) * m.sin(fi)

        self.ball_coord = [self.ball_coord_x, self.ball_coord_y, self.ball_coord_z]

        vrep.simxSetObjectPosition(self.clientID, self.handle_ball, -1, self.ball_coord, vrep.simx_opmode_oneshot_wait)


    def get_end_coord(self):
        sleep(0.15)
        errorCode, end_coord = vrep.simxGetObjectPosition(self.clientID, self.handle_end_pincher, -1,
                                                               vrep.simx_opmode_oneshot_wait)
        self.end_coord = list(end_coord)
        self.end_coord_x = self.end_coord[0]
        self.end_coord_y = self.end_coord[1]
        self.end_coord_z = self.end_coord[2]
        vrep.simxSetObjectPosition(self.clientID, self.handle_end_tester, -1, self.end_coord,
                                   vrep.simx_opmode_oneshot_wait)


    def distance(self):

        x1 = self.end_coord_x
        y1 = self.end_coord_y
        z1 = self.end_coord_z
        x2 = self.ball_coord_x
        y2 = self.ball_coord_y
        z2 = self.ball_coord_z

        self.dist = m.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)





    def get_obs(self):
        self.distance()
        dist = self.dist
        r1, alf1, fi1 = coord_to_polar(self.end_coord_x, self.end_coord_y, self.end_coord_z)
        r2, alf2, fi2 = coord_to_polar(self.ball_coord_x, self.ball_coord_y, self.ball_coord_z)

        result = np.array(
            [dist / 0.6, self.q1 / m.pi, 2 * self.q2 / m.pi, 2 * self.q3 / m.pi, 2 * self.q4 / m.pi,
             self.end_coord_x / 0.35, self.end_coord_y / 0.35, self.end_coord_z / 0.45, r1 / 0.45, alf1 / m.pi,
             fi1 / m.pi, self.ball_coord_x / 0.35, self.ball_coord_y / 0.35, self.ball_coord_z / 0.45, r2 / 0.45,
             alf2 / m.pi, fi2 / m.pi])

        return result

    def choice(self):
        a = []
        for i in itertools.product((-1, 0, 1), repeat=4):
            a.append(list(i))
        return a





'''
env = PXP_env()
print(len(env.choice()))





env.configurate()
print(env.reset())
'''

