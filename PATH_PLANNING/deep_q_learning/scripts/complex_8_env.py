
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
        self.action_space = ['u', 'd', 'l', 'r', 'ul', 'ur', 'dl', 'dr']
        self.n_actions = len(self.action_space)
        self.n_features = 2
        self._build_maze()
        self.X_ob = 0
        self.Y_ob = 0
        self.rect = np.array([0,0])
    
    def _build_maze(self):
        self.ground = np.tile(0,(X_LEN,Y_LEN))
        
        #hell
        self.hell_center = np.array([[0, 6],
                                    [0, 7],
                                    [0, 8],
                                    [0, 9],
                                    [0, 10],
                                    [0, 11],
                                    [0, 12],
                                    [0, 13],
                                    [0, 14],
                                    [0, 15],
                                    [0, 16],
                                    [0, 17],
                                    [0, 18],
                                    [0, 19],
                                    [1, 12],
                                    [1, 13],
                                    [1, 14],
                                    [1, 15],
                                    [1, 16],
                                    [1, 17],
                                    [1, 18],
                                    [1, 19],
                                    [2, 17],
                                    [2, 18],
                                    [2, 19],
                                    [3,17],
                                    [3,18],
                                    [3,19],
                                    [4,17],
                                    [4,18],
                                    [4,19],
                                    [5,17],
                                    [5,18],
                                    [5,19],
                                    [6,18],
                                    [6,19],
                                    [7,18],
                                    [7,19],
                                    [8,18],
                                    [8,19],
                                    [9,18],
                                    [9,19],
                                    [10,18],
                                    [10,19],
                                    [11,18],
                                    [11,19],
                                    [6,1],
                                    [6,2],
                                    [7,1],
                                    [7,2],
                                    [8,10],
                                    [9,9],
                                    [9,10],
                                    [9,11],
                                    [10,8],
                                    [10,9],
                                    [10,10],
                                    [10,11],
                                    [10,12],
                                    [11,9],
                                    [11,10],
                                    [11,11],
                                    [12,10],
                                    [13,0],
                                    [13,1],
                                    [14,0],
                                    [14,1],
                                    [15,0],
                                    [15,1],
                                    [16,0],
                                    [16,1],
                                    [17,0],
                                    [17,1],
                                    [18,0],
                                    [18,1],
                                    [18,2],
                                    [18,3],
                                    [18,4],
                                    [18,5],
                                    [18,6],
                                    [18,7],
                                    [19,0],
                                    [19,1],
                                    [19,2],
                                    [19,3],
                                    [19,4],
                                    [19,5],
                                    [19,6],
                                    [19,7],
                                    [19,8],
                                    [19,9],
                                    [19,10],
                                    [19,11],
                                    [19,12]])
        #ovel
        self.ovel_center = np.array([16,17])

        for i in range(np.size(self.hell_center)/2):
            x = self.hell_center[i, 0]
            y = self.hell_center[i, 1]
            self.ground[x, y] = -1
        
        x = self.ovel_center[0]
        y = self.ovel_center[1]
        self.ground[x, y] = 1
    
    def reset(self, isset = False, inputPosx = 0, inputPosy = 0):
        while True:
            if isset:
                if inputPosx > (X_LEN)*5 or inputPosx < 0 or inputPosy > (Y_LEN)*5 or inputPosx < 0:
                    self.rect = np.array([0, 0])
                else:
                    self.rect = np.array([int(inputPosx/5), int(inputPosy/5)])
                    #print self.rect
                    #print 'rect/5', inputPosx, inputPosy
            else:
                self.rect = np.array([np.random.randint(0, (X_LEN-1)),np.random.randint(0, (Y_LEN-1))])

            if self.ground[self.rect[0],self.rect[1]] == 0:
                break
        
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
        if action == 4: #up right
            if s[1] >= 1:
                base_action[1] -= 1
            if s[0] < (X_LEN - 1) :
                base_action[0] += 1
        elif action == 5:   # up left
            if s[1] >= 1:
                base_action[1] -= 1
            if s[0] > 1:
                base_action[0] -= 1
        elif action == 6:   # down right
            if s[1] < (Y_LEN - 1):
                base_action[1] += 1
            if s[0] < (X_LEN - 1) :
                base_action[0] += 1
        elif action == 7:   # down left
            if s[1] < (Y_LEN - 1):
                base_action[1] += 1
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