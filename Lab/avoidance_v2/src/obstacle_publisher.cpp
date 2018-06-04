//by Boxian Deng, Jun 03, 2018
//deal with obstacle position publisher
//used for circle obstacles
#include "avoidance_v2.h"
//messages
#include "geometry_msgs/PoseArray.h"

int_T main(int_T argc, char **argv)
{
    // Unused arguments
    (void)(argc);
    (void)(argv);

    ros::init(argc, argv, "obstacle");

    ros::NodeHandle obstacle_node;

    ros::Publisher obstacle_pub = obstacle_node.advertise<geometry_msgs::PoseArray>("obstaclePosition", 2);

    ros::Rate loop_rate(10);
    
    while (ros::ok()) {

        geometry_msgs::PoseArray pub_msg;

	    geometry_msgs::Pose p_obs_1;
	    p_obs_1.position.x = 30.0;
	    p_obs_1.position.y = 0.0;
	    pub_msg.poses.push_back(p_obs_1);
	    
	    geometry_msgs::Pose p_obs_2;
	    p_obs_2.position.x = 45.0;
	    p_obs_2.position.y = 0.0;
	    pub_msg.poses.push_back(p_obs_2);
	    
	    geometry_msgs::Pose p_obs_3;
	    p_obs_3.position.x = 60.0;
	    p_obs_3.position.y = 0.0;
	    pub_msg.poses.push_back(p_obs_3);

	    //ROS_INFO("[Publish] I published: the obstacles position is (%f, %f)", p_obs.position.x, p_obs.position.y);

        obstacle_pub.publish(pub_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}