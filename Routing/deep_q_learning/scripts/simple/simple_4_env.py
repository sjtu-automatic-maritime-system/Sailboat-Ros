
"""
explorer.
hells       [reward = -1].
paradise    [reward = +1].
ground      [reward = 0].

This script is the environment part of this example.
The RL is in RL_brain.py.
"""

import numpy as np
import time
import sys

X_LEN = 20
Y_LEN = 20
UNIT = 5 # 5m

class Maze:
    def __init__(self):
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.n_features = 2
        self._build_maze()
        self.X_ob = 0
        self.Y_ob = 0
        self.rect = np.array([0,0])
    
    def _build_maze(self):
        self.ground = np.tile(0,(X_LEN,Y_LEN))
        
        #hell
        self.hell_center = np.array([[12, 7],
                                    [5, 13],
                                    [8,4],
                                    [15,15]])
        #ovel
        self.ovel_center = np.array([18,18])

        for i in range(np.size(self.hell_center)/2):
            x = self.hell_center[i, 0]
            y = self.hell_center[i, 1]
            self.ground[x, y] = -1
        
        x = self.ovel_center[0]
        y = self.ovel_center[1]
        self.ground[x, y] = 1
    
    def reset(self, isset = False, inputPosx = 0, inputPosy = 0):
        if isset:
            if inputPosx > (X_LEN)*5 or inputPosx < 0 or inputPosy > (Y_LEN)*5 or inputPosx < 0:
                self.rect = np.array([0, 0])
            else:
                self.rect = np.array([int(inputPosx/5), int(inputPosy/5)])
                #print self.rect
                #print 'rect/5', inputPosx, inputPosy
        else:
            self.rect = np.array([np.random.randint(0, (X_LEN-1)),np.random.randint(0, (Y_LEN-1))])
        
        return 1.0*(self.rect - self.ovel_center)/(max(X_LEN,Y_LEN))
    
    def step(self, action):
        is_oval = False
        s = self.rect
        base_action = np.array([0, 0])
        
        if action == 0: #up
            if s[1] >= 1:
                base_action[1] -= 1
        elif action == 1:   # down
            if s[1] < (Y_LEN - 1):
                base_action[1] += 1
        elif action == 2:   # right
            if s[0] < (X_LEN - 1) :
                base_action[0] += 1
        elif action == 3:   # left
            if s[0] > 1:
                base_action[0] -= 1
        
        self.rect = self.rect + base_action

        x = self.rect[0]
        y = self.rect[1]
        #print self.ground
        if self.ground[x, y] == 1:
            reward = 1
            done = True
            is_oval = True
            print ('in oval')
        elif self.ground[x, y] == -1:
            reward = -1
            done = True
            print ('in hell')
        else:
            reward = 0
            done = False
        s_ = 1.0*(self.rect - self.ovel_center)/(max(X_LEN,Y_LEN))
        #print s_
        #print self.ground
        return s_, reward, done, is_oval