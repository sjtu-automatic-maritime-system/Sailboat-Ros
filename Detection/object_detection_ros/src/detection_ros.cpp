
#include "object_detection_ros/detection_ros.h"


//ball x 10   y 20
//     x 25.5 y -4.1
// 772.2589954342994 0 648.5 -54.05812968040097
// 0 772.2589954342994 482.5 0 
// 0 0 1 0

//1296x964

DetectionRos::DetectionRos(double ballR, double fov,bool gmapping)
    // : nh(_comm_nh),
    : it(nh)
{
    ROS_INFO("node init");
    if (gmapping){
        sub_image = nh.subscribe("/camera/image_raw", 2, &DetectionRos::detection_gmapping_cb, this);
    }
    else{
        sub_image = nh.subscribe("/camera/image_raw", 2, &DetectionRos::detection_cb, this);
    }
    
    gps = nh.subscribe("/sensor", 2, &DetectionRos::sensor_cb, this);
//    ros::Subscriber sub_image =  nh.subscribe("camera/image_raw/compressed", 2, &detection_cb);
    obj_pub = nh.advertise<geometry_msgs::PoseArray>("/object/pose", 2); 
    laser_pub = nh.advertise<sensor_msgs::LaserScan>("/base_scan",2);
    
    tld_pub = nh.advertise<tld_msgs::BoundingBox>("/tld_tracked_object",2);

    pub_img_edge = it.advertise("edges", 2);
    pub_img_dst = it.advertise("dst_circles", 2);
    
    posX = 0;
    posY = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;

    f = IMG_WIDTH/(2*std::tan(fov/(2*57.3)));
    
    ball_r = ballR;
    Num_ball = 1;

    particleFilter = new ParticleFilter();
    //average = new Average();
}

DetectionRos::~DetectionRos(){

}

//roll pitch yaw
void DetectionRos::roll_pitch_yaw_to_R(Vector3d E,Matrix3d &R){
    double sx = sin(E(0));
    double cx = cos(E(0));
    double sy = sin(E(1));
    double cy = cos(E(1));
    double sz = sin(E(2));
    double cz = cos(E(2));
    R(0,0) = cy*cz;
    R(0,1) = cz*sx*sy-cx*sz;
    R(0,2) = sx*sz+cx*cz*sy;
    R(1,0) = cy*sz;
    R(1,1) = cx*cz+sx*sy*sz;
    R(1,2) = cx*sy*sz-cz*sz;
    R(2,0) = -sy;
    R(2,1) = cy*sx;
    R(2,2) = cx*cy;
    //cout<<"R=\n"<<R<<endl;
}


void DetectionRos::detection_cb(const sensor_msgs::ImageConstPtr& img_in)
{
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in,"bgr8")->image.copyTo(src_img);
//    src_img = cv::imdecode(cv::Mat(img_in->data),3);//convert compressed image data to cv::Mat
//    cv::imshow("src", src_img);
//    cv::waitKey(5);
    cv::Mat src_ROI;
    //int src_ROI_p1_y = IMG_HEIGHT/3;
    int src_ROI_p1_y = 0;
    src_ROI = src_img(cv::Rect(0,src_ROI_p1_y,IMG_WIDTH,IMG_HEIGHT-src_ROI_p1_y));
    flip(src_ROI, src_ROI, -1);
    cv::Mat edge = detection::edgeDetection(src_ROI, 150, 200);
    std::vector<cv::Vec3f> circles = detection::circleDectection(src_ROI, edge);
    ROS_INFO("detected circles = %ld",circles.size());
    if (Num_ball < circles.size()){
        Num_ball = circles.size();
    }
    
    for (size_t i = 0; i < circles.size(); i++) {
        std::cout << "circle " << i+1 << " = " <<circles[i] << std::endl;
        double h_angle = std::atan((circles[i][0]-IMG_WIDTH/2)/f);
        double v_angle = std::atan((circles[i][1]-IMG_HEIGHT/2)/f);
        //std::cout << "h_angle = " << h_angle*57.3 << std::endl;
        //std::cout << "v_angle = " << v_angle*57.3 << std::endl;
        if (get_sensor){
            Matrix3d R_tmp;
            Matrix3d R_in_tmp;
            Vector3d E_tmp;
            R_tmp = Matrix3d::Zero(3,3);
            R_in_tmp = Matrix3d::Zero(3,3);
            E_tmp(0) = roll;
            E_tmp(1) = pitch;
            E_tmp(2) = 0;
            roll_pitch_yaw_to_R(E_tmp,R_tmp);
            R_in_tmp = R_tmp.inverse().eval();
            
            MatrixXd L_ship(3,1);
            MatrixXd L_world(3,1);
            L_ship(0,0) = f;
            L_ship(1,0) = circles[i][0]-IMG_WIDTH/2;
            L_ship(2,0) = circles[i][1]-IMG_HEIGHT/2;
            L_world = R_tmp*L_ship;
            // std::cout << "L_ship : " << L_ship << std::endl;
            // std::cout << "L_world : " << L_world << std::endl;
            
            double h_angle_final = std::atan((L_world(1,0))/L_world(0,0));
            double v_angle_final = std::atan((L_world(2,0))/L_world(0,0));
            //std::cout << "h_angle = " << h_angle*57.3 << std::endl;
            //std::cout << "v_angle = " << v_angle*57.3 << std::endl;

            // cal distance funcation 1
            double distance_cal = ball_r/circles[i][2]*std::sqrt(std::pow(circles[i][0]-IMG_WIDTH/2,2)+std::pow(circles[i][1]-IMG_HEIGHT/2,2)+std::pow(f,2));
            ROS_INFO("distance_cal = %f",distance_cal);
            
            double object_yaw = yaw + h_angle_final;
            double object_x = posX + distance_cal*cos(object_yaw);
            double object_y = posY + distance_cal*sin(object_yaw);

            ROS_INFO("object pos : ( %f , %f ) object_yaw : %f",object_x, object_y,object_yaw);

            double final_x,final_y,last_yaw;
            particleFilter->run(object_x, object_y, object_yaw, final_x, final_y, last_yaw);
            //average->run(object_x, object_y, object_yaw, final_x, final_y);
            double distance = std::sqrt(std::pow((posX-final_x),2.0)+std::pow((posY-final_y),2.0));
            ROS_INFO("distance_final = %f",distance);
            
            double ball_r_new = ball_r/distance_cal * distance;
            ROS_INFO("ball_r_new = %f",ball_r_new);
        }
        // cal angle
        

        //cal distance funcation 2
        /*
        MatrixXd L_tmp(3,1);
        L_tmp = L_world/f;
        double Z = 0.2;
        double X_cam = (Z)/L_tmp(2,0);
        double X = X_cam*L_tmp(0,0);
        double Y = X_cam*L_tmp(1,0);
         
        double Z_cal = distance_cal*L_tmp(2,0);
        Z_cal = Z_cal / std::sqrt(std::pow(L_tmp(0,0),2)+std::pow(L_tmp(1,0),2));
        std::cout << "Z_cal = " << Z_cal <<std::endl;
        //double distance_2 = std::sqrt(std::pow(X,2)+std::pow(Y,2));
        //std::cout << "distance_2 = " << distance_2 <<std::endl;
        Vector3d E_tmp_2;
        Matrix3d R_tmp_2;
        E_tmp_2(0) = 0;
        E_tmp_2(1) = 0;
        E_tmp_2(2) = yaw;
        roll_pitch_yaw_to_R(E_tmp_2,R_tmp_2); 
        */
    }

    //average->publish();
    if (get_sensor){
        objectPoseArray object_pose_array;
        particleFilter->publish(object_pose_array);

        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "object";
        for (int i = 0; i < object_pose_array.poseArray.size(); i++)
        {  
            double x = object_pose_array.poseArray[i].x;  
            double y = object_pose_array.poseArray[i].y;  
            geometry_msgs::Pose p;     
            p.position.x = x;  
            p.position.y = y;  
            p.position.z = 0;  
            pose_array.poses.push_back(p);  
        }
        obj_pub.publish(pose_array);
    }

    sensor_msgs::ImagePtr edge_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", edge).toImageMsg();
    pub_img_edge.publish(edge_msg);

    sensor_msgs::ImagePtr dst_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_ROI).toImageMsg();
    pub_img_dst.publish(dst_msg);

    tld_msgs::BoundingBox tldMsg;
    if (circles.size()>0){
        
        tldMsg.confidence = 1;
    }
    else{
        tldMsg.confidence = 0;
    }
    tld_pub.publish(tldMsg);

    get_sensor = false;
}


void DetectionRos::detection_gmapping_cb(const sensor_msgs::ImageConstPtr& img_in){
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in,"bgr8")->image.copyTo(src_img);
//    src_img = cv::imdecode(cv::Mat(img_in->data),3);//convert compressed image data to cv::Mat
//    cv::imshow("src", src_img);
//    cv::waitKey(5);
    cv::Mat src_ROI;
    //int src_ROI_p1_y = IMG_HEIGHT/3;
    int src_ROI_p1_y = 0;
    src_ROI = src_img(cv::Rect(0,src_ROI_p1_y,IMG_WIDTH,IMG_HEIGHT-src_ROI_p1_y));
    cv::Mat edge = detection::edgeDetection(src_ROI, 150, 200);
    std::vector<cv::Vec3f> circles = detection::circleDectection(src_ROI, edge);
    ROS_INFO("detected circles = %ld",circles.size());

    float ranges_size = 81;
    float angle_increment = (80/57.3)/(ranges_size-1);

    sensor_msgs::LaserScan laser_msg;
    laser_msg.header = img_in->header;
    laser_msg.angle_min = -40/57.3;
    laser_msg.angle_max = 40/57.3;
    laser_msg.angle_increment = angle_increment;
    laser_msg.time_increment = 0;
    laser_msg.scan_time = 1/10;
    laser_msg.range_min = 2;
    laser_msg.range_max = 40.0;

    laser_msg.ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

    for (size_t i = 0; i < circles.size(); i++) {
        float h_angle = std::atan((circles[i][0]-IMG_WIDTH/2)/f);
        float v_angle = std::atan((circles[i][1]-IMG_HEIGHT/2)/f);
        //std::cout << "h_angle = " << h_angle*57.3 << std::endl;
        //std::cout << "v_angle = " << v_angle*57.3 << std::endl;
        // cal angle
        Matrix3d R_tmp;
        Matrix3d R_in_tmp;
        Vector3d E_tmp;
        R_tmp = Matrix3d::Zero(3,3);
        R_in_tmp = Matrix3d::Zero(3,3);
        E_tmp(0) = roll;
        E_tmp(1) = pitch;
        E_tmp(2) = 0;
        roll_pitch_yaw_to_R(E_tmp,R_tmp);
        R_in_tmp = R_tmp.inverse().eval();
        
        MatrixXd L_ship(3,1);
        MatrixXd L_world(3,1);
        L_ship(0,0) = f;
        L_ship(1,0) = circles[i][0]-IMG_WIDTH/2;
        L_ship(2,0) = circles[i][1]-IMG_HEIGHT/2;
        L_world = R_tmp*L_ship;
        // std::cout << "L_ship : " << L_ship << std::endl;
        // std::cout << "L_world : " << L_world << std::endl;
        float h_angle_final = std::atan((L_world(1,0))/L_world(0,0));
        float v_angle_final = std::atan((L_world(2,0))/L_world(0,0));
        std::cout << "h_angle = " << h_angle*57.3 << std::endl;
        //std::cout << "v_angle = " << v_angle*57.3 << std::endl;

        // cal distance funcation 1
        float distance_cal = ball_r/circles[i][2]*std::sqrt(std::pow(circles[i][0]-IMG_WIDTH/2,2)+std::pow(circles[i][1]-IMG_HEIGHT/2,2)+std::pow(f,2));
        ROS_INFO("distance_cal = %f",distance_cal);

        double object_yaw = yaw + h_angle_final;
        double object_x = posX + distance_cal*cos(object_yaw);
        double object_y = posY + distance_cal*sin(object_yaw);

        ROS_INFO("object pos : ( %f , %f ) object_yaw : %f",object_x, object_y,object_yaw);

        float range = ball_r/distance_cal;
        float range_max = 40/57.3 + h_angle_final + range;
        float range_min = 40/57.3 + h_angle_final - range;

        int index_max = range_max/angle_increment;
        int index_min = range_min/angle_increment;
        ROS_INFO("index_max = %d",index_max);
        ROS_INFO("index_min = %d",index_min);

        if (index_min<0){
            index_min = 0;
        }
        if (index_max>80){
            index_max = 80;
        }

        if (distance_cal > laser_msg.range_min and distance_cal < laser_msg.range_max){
            
            for (int i = index_min; i < index_max+1; i++){
                //ROS_INFO("index = %d",i);
                laser_msg.ranges[i] = distance_cal;
            }
        }
    }

    laser_pub.publish(laser_msg);

}

void DetectionRos::sensor_cb(const sailboat_message::Sensor_msg::ConstPtr& msg){
    get_sensor = true;
    //ROS_INFO("get sensor");
    posX = msg->Posx;
    posY = msg->Posy;
    roll = msg->Roll;
    pitch = msg->Pitch;
    yaw = msg->Yaw;
    double distance = std::sqrt(std::pow((posX-25.5),2.0)+std::pow((posY+4.1),2.0));
    //std::cout << "sensor distance = " << distance << "; posX = "<<posX<<"; posY = "<<posY<< std::endl;
}

