#!/usr/bin/env python
import rospy
from sailboat_message.msg import Sensor_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped 

from complex_8_env import Maze
from natural_dqn_brain import DeepQNetwork
from double_dqn_brain import DoubleDQN
from prioritized_replay_brain import DQNPrioritizedReplay
import numpy as np
import os

getMsg = 0
posx = 0
posy = 0

#ros
def callback(sensorMsg):
    global posx
    global posy
    global getMsg
    getMsg = 1
    posx = float(sensorMsg.Posx)
    posy = float(sensorMsg.Posy)



def get_route_maze():
    rospy.init_node('dqn_path_planning', anonymous=True)
    pub = rospy.Publisher('planned_path',Path, queue_size=2)
    rospy.Subscriber("sensor", Sensor_msg, callback)
    rate = rospy.Rate(1) # 0.025hz

    RL.reload_model()
    while not rospy.is_shutdown():
        inputx = int(posx/5)
        inputy = int(posy/5)
        print inputx," ~ ",inputy
        is_oval = False
        is_done = False

        while ~is_oval:
            routeX_old = np.array([])
            routeY_old = np.array([])
            for step in range(1):
                #print posx," ~ ",posy
                observation = env.reset(True, posx, posy)
                if step != 0 and is_oval:
                    routeX_old = routeX
                    routeY_old = routeY
                routeX = np.array([])
                routeY = np.array([])
                step_one = 0
                while ~is_done:
                    # fresh env
                    # RL choose action based on observation
                    action = RL.choose_action(observation, False)
                    # RL take action and get next observation and reward
                    observation_, reward, is_done, is_oval = env.step(action)
                    # swap observation
                    observation = observation_
                    pathX = observation[0] * 100 + 18 * 5
                    pathY = observation[1] * 100 + 18 * 5
                    routeX = np.append(routeX, [pathX])
                    routeY = np.append(routeY, [pathY])
                    print pathX,' ~ ',pathY
                    if step_one > 100:
                        break
                    # break while loop when end of this episode
                    if is_done:
                        if step != 0 and is_oval:
                            routeX, routeY = modify_path(routeX, routeY)
                            if np.size(routeX) > np.size(routeX_old) and np.size(routeX_old) != 0:
                                routeX = routeX_old
                                routeY = routeY_old
                            routeX_old = routeX
                            routeY_old = routeY
                        break
                    step_one += 1 

            if is_oval:
                
                # print(routeX)
                # print(routeY)
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
    model_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/natural_dqn/natural_dqn_model_20171222.ckpt"
    #model_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/double_dqn/double_dqn_model_20171222.ckpt"
    #model_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/prioritized_replay/prioritized_replay_model_20171222.ckpt"
    #DeepQNetwork
    #DoubleDQN
    #DQNPrioritizedReplay
    RL = DeepQNetwork(env.n_actions, env.n_features, model_path,
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      replace_target_iter=200,
                      memory_size=4000,
                      # output_graph=True
                      )

    get_route_maze()
    #RL.plot_cost()
