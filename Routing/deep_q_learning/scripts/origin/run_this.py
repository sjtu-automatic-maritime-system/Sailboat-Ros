#!/usr/bin/env python
import rospy
from sailboat_message.msg import Sensor_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped 

from maze_env import Maze
from RL_brain import DeepQNetwork
import numpy as np
import os

getMsg = 0
posx = 1
posy = 1

#ros
def callback(sensorMsg):
    global posx
    global posy
    global getMsg
    getMsg = 1
    posx = float(sensorMsg.Posx)
    posy = float(sensorMsg.Posy)



def run_maze():
    step = 0
    RL.reload_model()
    for episode in range(4000):
        # initial observation

        observation = env.reset()

        while True:
            # fresh env
            env.render()

            # RL choose action based on observation
            action = RL.choose_action(observation, True)

            # RL take action and get next observation and reward
            observation_, reward, done, is_oval = env.step(action)

            RL.store_transition(observation, action, reward, observation_)

            if (step > 200) and (step % 5 == 0):
                RL.learn()

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done:
                break
            step += 1

    # save the model
    RL.save_model()

    # end of q learn
    print('finish DQN')
    env.destroy()

def test_maze():

    RL.reload_model()
    for episode in range(10):
        # initial observation
        observation = env.reset(True, episode, episode)
        while True:
            # fresh env
            env.render()
            # RL choose action based on observation
            action = RL.choose_action(observation, False)
            # RL take action and get next observation and reward
            observation_, reward, done, is_oval = env.step(action)
            # swap observation
            observation = observation_
            print observation * 100 + 18 * 5
            # break while loop when end of this episode
            if done:
                break
    print('finish test')
    env.destroy()

def get_route_maze():
    rospy.init_node('dqn_path_planning', anonymous=True)
    pub = rospy.Publisher('planned_path',Path, queue_size=2)
    rospy.Subscriber("sensor", Sensor_msg, callback)
    rate = rospy.Rate(0.02) # 0.025hz

    RL.reload_model()
    while not rospy.is_shutdown():
        inputx = int(posx/10)
        inputy = int(posy/10)
        is_oval = False
        is_done = False
        while ~is_oval:
            for step in range(5):
                observation = env.reset(True, inputx, inputy)
                if step != 0 and is_oval:
                    routeX_old = routeX
                    routeY_old = routeY
                routeX = np.array([])
                routeY = np.array([])
                while ~is_done:
                    # fresh env
                    env.render()
                    # RL choose action based on observation
                    action = RL.choose_action(observation, False)
                    # RL take action and get next observation and reward
                    observation_, reward, is_done, is_oval = env.step(action)
                    # swap observation
                    observation = observation_
                    pathX = observation[0] * 200 + 18 * 10
                    pathY = observation[1] * 200 + 18 * 10
                    routeX = np.append(routeX, [pathX])
                    routeY = np.append(routeY, [pathY])
                    # break while loop when end of this episode
                    if is_done:
                        if step != 0 and is_oval:
                            routeX, routeY = modify_path(routeX, routeY)
                            if np.size(routeX) > np.size(routeX_old):
                                routeX = routeX_old
                                routeY = routeY_old
                            routeX_old = routeX
                            routeY_old = routeY
                        break
            if is_oval:
                
                print(routeX)
                print(routeY)
                PathMsg = Path()
                PathMsg.header.stamp = rospy.Time.now()
                PathMsg.header.frame_id = "odom"
                
                for i in range(np.size(routeX)):
                    poseMsg = PoseStamped()
                    poseMsg.pose.position.x = routeX[i]
                    poseMsg.pose.position.y = routeY[i]
                    PathMsg.poses.append(poseMsg)
                pub.publish(PathMsg)
                break
        rate.sleep()

    print('get routeX and routeY')
    env.destroy()
    rospy.spin()


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
    model_path = os.path.dirname(os.path.abspath(__file__)) + "/model20171219.ckpt"
    RL = DeepQNetwork(env.n_actions, env.n_features, model_path,
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      replace_target_iter=200,
                      memory_size=2000,
                      # output_graph=True
                      )
    env.after(100, get_route_maze)
    env.mainloop()
    #RL.plot_cost()