//
// Created by hywel on 17-12-20.
//

#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "aruco_ros/simple_single.h"

namespace aruco_ros {

    class ArucoSingle : public nodelet::Nodelet {
    public:
        ArucoSingle() {}

        void onInit() {
            ros::NodeHandle node = getNodeHandle();
            ros::NodeHandle pnode = getPrivateNodeHandle();
            aruco = new ArucoSimple(node,pnode);
        }

        ~ArucoSingle() {
            if (aruco) delete aruco;
        }

    private:
        ArucoSimple *aruco;
    };
};

PLUGINLIB_DECLARE_CLASS(aruco_ros, ArucoSingle, aruco_ros::ArucoSingle, nodelet::Nodelet);



