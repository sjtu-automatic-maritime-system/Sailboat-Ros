//
// Created by hywel on 3/13/18.
//

#define PI 3.1415926

#include <cmath>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include "sailboat_message/Sensor_msg.h"


struct imu{
    double roll;
    double pitch;
    double yaw;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double angular_vel_x;
    double angular_vel_y;
    double angular_vel_z;
    double linear_acc_x;
    double linear_acc_y;
    double linear_acc_z;
};

struct euler{
    double X;
    double Y;
    double Z;
};

struct gps_fix{
    double latitude;
    double longitude;
};


static double SensorMsg[10];

static double lat_orgin = -30.0602750*3.1415926/180;
static double lon_orgin = -51.1737670*3.1415926/180;


euler QuaternionToEuler(double w, double x,double y,double z) // Z-Y-X Euler angles
{
    euler Euler;
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;

    double TEST = w*y - x*z;

    if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
    {
        int sign;
        if(TEST >= 0)
            sign = 1;
        else
            sign = -1;

        Euler.Z = -2 * sign * (double)atan2(x, w); // yaw

        Euler.Y = sign * (PI / 2.0); // pitch

        Euler.X = 0; // roll

    }
    else
    {
        Euler.X = atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z);
        Euler.Y = asin(-2 * (x*z - w*y));
        Euler.Z = atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z);
    }

    return Euler;
}


double Limiting(double data){
    while (data > PI or data <-PI){
        if(data>PI)
            data = data - 2*PI;
        if(data<-PI)
            data = data + 2*PI;
    }
    return data;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

    imu Imu;
    geometry_msgs::Quaternion quaternion;
    geometry_msgs::Vector3 ang_v;
    geometry_msgs::Vector3 lin_a;

    quaternion = msg->orientation;
    ang_v = msg->angular_velocity;
    lin_a = msg->linear_acceleration;

    Imu.orientation_w = quaternion.w;
    Imu.orientation_x = quaternion.x;
    Imu.orientation_y = quaternion.y;
    Imu.orientation_z = quaternion.z;
    Imu.angular_vel_x = ang_v.x;
    Imu.angular_vel_y = ang_v.y;
    Imu.angular_vel_z = ang_v.z;
    Imu.linear_acc_x = lin_a.x;
    Imu.linear_acc_y = lin_a.y;
    Imu.linear_acc_z = lin_a.z;

    euler Euler;
    Euler = QuaternionToEuler(Imu.orientation_w,Imu.orientation_x,Imu.orientation_y,Imu.orientation_z);

    SensorMsg[2] = Imu.angular_vel_x;
    SensorMsg[3] = -Imu.angular_vel_z;

    SensorMsg[6] = Euler.X;
    SensorMsg[7] = PI/2 - Euler.Z;
    SensorMsg[7] = Limiting(SensorMsg[7]);
}


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    gps_fix Gps_Fix;
    Gps_Fix.latitude = msg->latitude;
    Gps_Fix.longitude = msg->longitude;

    double latitude = Gps_Fix.latitude*3.1415926/180;
    double longitude = Gps_Fix.longitude*3.1415926/180;

    double d_lat = latitude - lat_orgin;
    double d_lon = longitude - lon_orgin;

    double MACRO_AXIS = 6378137.0; // 赤道圆的平均半径
    double MINOR_AXIS = 6356752.0; // 半短轴的长度，地球两极距离的一半

    double a1 = 6378137.0;
    double e_2 = 6.69437999014e-3;
    double r1 = a1 * (1 - e_2) / pow((1 - e_2 * pow((sin(lat_orgin)), 2)) , 1.5);
    double r2 = a1 / sqrt(1 - e_2 * pow((sin(lat_orgin)) , 2));

    double north = r1*d_lat;
    double east = r2*cos(lat_orgin)*d_lon;

    double a = pow(MACRO_AXIS, 2.0);
    double b = pow(MINOR_AXIS, 2.0);
    double c = pow(tan(lat_orgin), 2.0);
    double d = pow(1/tan(lat_orgin),2.0);
    double x = a/sqrt(a + b*c);
    double y = b/sqrt(b + a*d);

    double e = pow(tan(latitude), 2.0);
    double f = pow(1/tan(latitude), 2.0);

    double m = a/sqrt(a + b*e);
    double n = b/sqrt(b + a*f);

    double turnY = sqrt(pow(x-m,2.0)+pow(y-n,2.0));

    double turnX = a/sqrt(a + b*c)* (longitude - lon_orgin);

    std::cout<<"turnX: "<< turnX << " " << north << std::endl;
    std::cout<<"turnY: "<< turnY << " " << east << std::endl;

    SensorMsg[4] = north;
    SensorMsg[5] = east;

}





void windCallback(const sailboat_message::Wind_Simulation_msg::ConstPtr& msg){
    double TWA = msg->TWA;
    double TWS = msg->TWS;
    SensorMsg[8] = PI + TWA - SensorMsg[7];
    SensorMsg[9] = TWS; //todo

    SensorMsg[8] = Limiting(SensorMsg[8]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_simulation");

    ros::NodeHandle nh;
    ros::Publisher sensor_pub;

    sensor_pub = nh.advertise<sailboat_message::Sensor_msg>("sensor", 2);

    ros::Subscriber imu_sub = nh.subscribe("imu", 2, &imuCallback);
    ros::Subscriber gps_sub = nh.subscribe("gps/fix", 2, &gpsCallback);
    ros::Subscriber wind_sub = nh.subscribe("wind", 2, &windCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        sailboat_message::Sensor_msg msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "sensor";
        msg.ux =   SensorMsg[0];
        msg.vy =   SensorMsg[1];
        msg.gx =   SensorMsg[2];
        msg.gz =   SensorMsg[3];
        msg.Posx = SensorMsg[4];
        msg.Posy = SensorMsg[5];
        msg.Roll = SensorMsg[6];
        msg.Yaw =  SensorMsg[7];
        msg.AWA =  SensorMsg[8];
        msg.AWS =  SensorMsg[9];

        sensor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}