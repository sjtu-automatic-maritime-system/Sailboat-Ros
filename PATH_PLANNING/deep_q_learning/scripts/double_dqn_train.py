#!/usr/bin/env python
from complex_8_env import Maze
from double_dqn_brain import DoubleDQN
import numpy as np
import os
import datetime

MEMORY_SIZE = 4000
def run_maze():
    starttime = datetime.datetime.now()
    step = 0
    for episode in range(8000):
        # initial observation

        observation = env.reset()

        while True:
            # fresh env
            # RL choose action based on observation
            action = RL.choose_action(observation, True)
            # RL take action and get next observation and reward
            observation_, reward, done, is_oval = env.step(action)

            RL.store_transition(observation, action, reward, observation_)

            if (step >  MEMORY_SIZE):
                RL.learn()

            # swap observation
            observation = observation_

            #pathX = observation[0] * 100 + 18 * 5
            #pathY = observation[1] * 100 + 18 * 5
            #print ("x~y  ", pathX,"~",pathY)

            # break while loop when end of this episode
            if done:
                break
            step += 1
    # save the model
    RL.save_model()
    # end of q learn
    #long running

    endtime = datetime.datetime.now()

    print ("run time: ",1.0*(endtime - starttime).seconds)
    print('finish double DQN training')

def analyze():
    starttime = datetime.datetime.now()
    step = 0
    i_suc = 0
    if_succeed = np.array([])
    num_point = np.array([])
    start_posx = np.array([])
    start_posy = np.array([])

    RL.reload_model()
    for episode in range(100):
        # initial observation
        observation = env.reset()

        start_posx = np.append(start_posx, observation[0])
        start_posy = np.append(start_posy, observation[1])

        routeX = np.array([])
        routeY = np.array([])
        while True:
            # fresh env
            # RL choose action based on observation
            action = RL.choose_action(observation, False)
            # RL take action and get next observation and reward
            observation_, reward, done, is_oval = env.step(action)
            # swap observation
            observation = observation_

            pathX = observation[0] * 100 + 18 * 5
            pathY = observation[1] * 100 + 18 * 5
            routeX = np.append(routeX, [pathX])
            routeY = np.append(routeY, [pathY])
            routeX, routeY = modify_path(routeX, routeY)
            
            # break while loop when end of this episode
            if done:
                num_point = np.append(num_point,np.size(routeX))
                if is_oval:
                    if_succeed = np.append(if_succeed, 1)
                    i_suc += 1
                else:
                    if_succeed = np.append(if_succeed, 0)
                break
            step += 1
    
    correct_rate = 1.0*i_suc/100

    print ("correct_rate: ", correct_rate)

    print ("start_posx: ", start_posx)
    print ("start_posy: ", start_posy)
    print ("if_succeed: ", if_succeed)
    print ("num_point: ", num_point)

    endtime = datetime.datetime.now()

    print ("run time: ",1.0*(endtime - starttime).seconds)
    # end of q learn
    
    print('finish double DQN analyze')


def find_same(routeX, routeY):
    i1 = -1
    i2 = -1
    for i in range(np.size(routeX)):
        for j in range(i+1, np.size(routeX)):
            if routeX[i] == routeX[j] and routeY[i] == routeY[j]:
                #print i, j
                i1 = i
                i2 = j
                return True, i1, i2
    return False, i1, i2


def modify_path(routeX, routeY):
    isFind, i1, i2 = find_same(routeX, routeY)
    while isFind:
        routeX = np.append(routeX[:i1], routeX[i2:])
        routeY = np.append(routeY[:i1], routeY[i2:])
        #print(routeX)
        #print(routeY)
        isFind, i1, i2 = find_same(routeX, routeY)
    return routeX, routeY


if __name__ == "__main__":
    # maze game
    env = Maze()
    model_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/double_dqn/double_dqn_model_20171222.ckpt"
    RL = DoubleDQN(env.n_actions, env.n_features, model_path,
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      replace_target_iter=200,
                      memory_size=4000,
                      e_greedy_increment=0.001,
                      double_q=True,
                      output_graph=True
                      )
    run_maze()
    analyze()
    RL.plot_cost()
