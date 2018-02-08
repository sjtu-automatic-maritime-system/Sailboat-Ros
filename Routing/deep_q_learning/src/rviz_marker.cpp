#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rviz_marker");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);

    //uint32_t shape = visualization_msgs::Marker::CUBE;

    while(ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        //marker.lifetime = ros::Duration();

        while(marker_pub.getNumSubscribers() < 1)
        {
            if(!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        int object[94][2] = {
            {0, 6},
            {0, 7},
            {0, 8},
            {0, 9},
            {0, 10},
            {0, 11},
            {0, 12},
            {0, 13},
            {0, 14},
            {0, 15},
            {0, 16},
            {0, 17},
            {0, 18},
            {0, 19},
            {1, 12},
            {1, 13},
            {1, 14},
            {1, 15},
            {1, 16},
            {1, 17},
            {1, 18},
            {1, 19},
            {2, 17},
            {2, 18},
            {2, 19},
            {3,17},
            {3,18},
            {3,19},
            {4,17},
            {4,18},
            {4,19},
            {5,17},
            {5,18},
            {5,19},
            {6,18},
            {6,19},
            {7,18},
            {7,19},
            {8,18},
            {8,19},
            {9,18},
            {9,19},
            {10,18},
            {10,19},
            {11,18},
            {11,19},
            {6,1},
            {6,2},
            {7,1},
            {7,2},
            //
            {9,10},
            {10,9},
            {10,10},
            {10,11},
            {11,9},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},

            // {8,10},
            // {9,9},
            // {9,10},
            // {9,11},
            // {10,8},
            // {10,9},
            // {10,10},
            // {10,11},
            // {10,12},
            // {11,9},
            // {11,10},
            // {11,11},
            // {12,10},
            {13,0},
            {13,1},
            {14,0},
            {14,1},
            {15,0},
            {15,1},
            {16,0},
            {16,1},
            {17,0},
            {17,1},
            {18,0},
            {18,1},
            {18,2},
            {18,3},
            {18,4},
            {18,5},
            {18,6},
            {18,7},
            {19,0},
            {19,1},
            {19,2},
            {19,3},
            {19,4},
            {19,5},
            {19,6},
            {19,7},
            {19,8},
            {19,9},
            {19,10},
            {19,11},
            {19,12}};

        for (int i = 0; i < 94; ++i)
        {  
            float x = object[i][1]*5;  
            float y = object[i][2]*5;  
    
            geometry_msgs::Point p;  
            p.x = x;  
            p.y = y;  
            p.z = 0;  
   
            marker.points.push_back(p);  
        }

        marker.lifetime = ros::Duration();


        marker_pub.publish(marker);

        r.sleep();
    }

}