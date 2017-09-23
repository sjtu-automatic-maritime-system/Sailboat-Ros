#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "tld_msgs/BoundingBox.h"
#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"
#include "geometry_msgs/PointStamped.h"

#define IMG_WIDTH 1296
#define IMG_HEIGHT 964
#define FOV 100 //field of view 100 deg
#define DISTANCE_TO_BOW 0.2 // m
#define PIXELS_TO_BOW 50 // pixels

#define FOCAL_LENGTH 820
#define DISTANCE_CAM_TO_WATER 0.4 // distance from obstacle plane to camera center
#define BOAT_LENGTH 1.5  // m


using namespace std;
//using namespace cv;
using Eigen::VectorXd;
using Eigen::MatrixXd;

image_transport::Publisher pub_img;

ros::Publisher obs_boat_pub;
ros::Publisher obs_ground_pub;
Eigen::Matrix4d T_boat_to_ground;

MatrixXd cameraMatrix(3, 3);
MatrixXd cameraMatrix_inv(3, 3);
MatrixXd K(4, 4);
Eigen::Matrix4d T_boat_to_cam(4, 4);
Eigen::Matrix4d T_cam_to_boat(4, 4);
Eigen::Matrix4d P_boat_to_img(4, 4);
Eigen::Matrix3d M_Mat(3, 3);
Eigen::Matrix3d M_Mat_inv(3, 3);
Eigen::VectorXd p_4(3);
Eigen::VectorXd C_tidle(3);
Eigen::MatrixXd P_ground_to_img(4, 4);
Eigen::Vector4d p_boat_reverse;
Eigen::Vector3d bbox_bottom;


void get_camera_info() {
//  camera info from file ../camera_info/0.yaml
//  camera matrix: [816.118268598647, 0, 680.6511245884145, 0, 822.0196620588329, 458.230641061779, 0, 0, 1]
    cameraMatrix << 816.118268598647, 0.000000, 680.6511245884145,
            0.000000, 822.0196620588329, 458.230641061779,
            0, 0, 1;

    K << 816.118268598647, 0.000000, 680.6511245884145, 0,
            0.000000, 822.0196620588329, 458.230641061779, 0,
            0, 0, 1, 0,
            0, 0, 0, 0;
    cameraMatrix_inv = cameraMatrix.inverse();
}

// not right just like a shit
//Eigen::Vector4d get_3d_point_from_pixel(Eigen::Vector3d pixel) {
//    Eigen::Vector3d p_camera = cameraMatrix_inv * pixel;
////    std::cout << "test" << p_camera[0] << std::endl;
//    Eigen::Vector4d h_p_camera;
//    h_p_camera << p_camera[0], p_camera[1], p_camera[2], 1;  //??todo not right
//    Eigen::Vector4d p_boat = T_cam_to_boat * h_p_camera;
//    Eigen::Vector4d h_camera_centor;
//    h_camera_centor << 0, 0, 0, 1;
//    Eigen::Vector4d c_boat = T_cam_to_boat * h_camera_centor;
//
//    double lambda = (0 - c_boat[2]) / p_boat[2];
//    double x = c_boat[0] + lambda * p_boat[0];
//    double y = c_boat[1] + lambda * p_boat[1];
//    Eigen::Vector4d p;
//    p << x, y, 0, 1;
//    return p;
//}

Eigen::Vector4d get_3d_point_from_pixel(Eigen::Vector3d pixel) {
    Eigen::Vector3d X_tidle = M_Mat_inv * pixel;
    double mue_N = -C_tidle[2] / X_tidle[2];
    Eigen::Vector4d p;
    double x = mue_N * X_tidle[0] + C_tidle[0];
    double y = mue_N * X_tidle[1] + C_tidle[1];
    double z = mue_N * X_tidle[2] + C_tidle[2];

    p << x, y, z, 1;
    return p;
}


Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
    return rz * ry * rx;
}

void get_tf_boat_camera() {
    // father-frame:camera, child-frame:boat  roll: zyx:90(yaw),-90(pitch),0(roll)
    // translation: 0, 0.3(DISTANCE_CAM_TO_WATER), -0.7(BOAT_LENGTH/2)
    // NOTE: boat_coord: x-forward, y-right, z-down, origin point in the water plane
    Eigen::Affine3d r = create_rotation_matrix(0, -M_PI_2, -M_PI_2);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0, DISTANCE_CAM_TO_WATER, -BOAT_LENGTH)));
//    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0, 0, -BOAT_LENGTH / 2)));
    T_boat_to_cam = (t * r).matrix();
    T_cam_to_boat = T_boat_to_cam.inverse();

    P_boat_to_img = K * T_boat_to_cam;
    std::cout << "P_boat_to_img: " << P_boat_to_img << std::endl;

    // reverse procedure
    M_Mat = P_boat_to_img.block(0, 0, 3, 3);
    std::cout << "M_Mat: " << M_Mat << std::endl;
    M_Mat_inv = M_Mat.inverse();
    p_4 = P_boat_to_img.block(0, 3, 3, 1);
    std::cout << "p_4: " << p_4 << std::endl;

    C_tidle = -M_Mat_inv * p_4;

}

void get_tf_boat_camera(double roll) {
    // father-frame:camera, child-frame:boat  roll: zyx:90(yaw),-90(pitch),0(roll)
    // translation: 0, 0.3(DISTANCE_CAM_TO_WATER), -0.7(BOAT_LENGTH/2)
    // NOTE: boat_coord: x-forward, y-right, z-down, origin point in the water plane
    Eigen::Affine3d r = create_rotation_matrix(roll, -M_PI_2, -M_PI_2);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0, DISTANCE_CAM_TO_WATER, -BOAT_LENGTH)));
//    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0, 0, -BOAT_LENGTH / 2)));
    T_boat_to_cam = (t * r).matrix();
    T_cam_to_boat = T_boat_to_cam.inverse();

    P_boat_to_img = K * T_boat_to_cam;
//    std::cout << "P_boat_to_img: " << P_boat_to_img << std::endl;

    // reverse procedure
    M_Mat = P_boat_to_img.block(0, 0, 3, 3);
//    std::cout << "M_Mat: " << M_Mat << std::endl;
    M_Mat_inv = M_Mat.inverse();
    p_4 = P_boat_to_img.block(0, 3, 3, 1);
//    std::cout << "p_4: " << p_4 << std::endl;

    C_tidle = -M_Mat_inv * p_4;

}


//
void bbox_cb(const tld_msgs::BoundingBoxConstPtr &bbox_msg) {
    if (bbox_msg->confidence > 0) {
        // x, y bbox upper left corner coord
        int x = bbox_msg->x;
        int y = bbox_msg->y;
        int height = bbox_msg->height;
        int width = bbox_msg->width;
        // x_center y_center center point of bbox
        int x_center = x + width / 2;
        int y_center = y + height / 2;
        int y_bottom = y + height;
        std::cout << "bbox_x: " << x_center << "  bbox_y: " << y_bottom << std::endl;

        // soluton1: use pixel postion to get distance and direction, not good
        //    double direction = (x_center - IMG_HEIGHT / 2) / (double) IMG_WIDTH * FOV / 57.3;
        //    double distance = (IMG_HEIGHT - y_center) / (double) PIXELS_TO_BOW * DISTANCE_TO_BOW;

        // solution2: use similiar triangle to get direction and distance, not good
        //    double direction = atan((x_center - IMG_WIDTH / 2) / (double) FOCAL_LENGTH);
        //    double distance = (double) FOCAL_LENGTH * DISTANCE_CAM_TO_WATER / (double) abs((y_bottom - IMG_HEIGHT / 2));
        //    std::cout << "direction: " << direction << "  distance: " << distance << std::endl;

        // solution3: use intrinsic projection procedure to get position, trying...
        bbox_bottom << x_center, y_bottom, 1;
        p_boat_reverse = get_3d_point_from_pixel(bbox_bottom);
        if (p_boat_reverse[0] < 0) {
//            p_boat_reverse[0] = -p_boat_reverse[0];
//            p_boat_reverse[1] = -p_boat_reverse[1];
            p_boat_reverse = -p_boat_reverse;

        }

//        if (p_boat_reverse[0] > 0) {
        if (1) {
            geometry_msgs::PointStamped obs_pos;
            obs_pos.header = bbox_msg->header;
            obs_pos.header.frame_id = "wtst";
            //    obs_pos.point.x = distance * cos(direction);
            //    obs_pos.point.y = distance * sin(direction);
            obs_pos.point.x = p_boat_reverse[0];
            obs_pos.point.y = p_boat_reverse[1];
            obs_pos.point.z = 0;
            std::cout << "obs_pos x: " << obs_pos.point.x << "  y: " << obs_pos.point.y << endl;
            obs_boat_pub.publish(obs_pos);


            VectorXd h_obs_boat(4, 1);
            h_obs_boat << obs_pos.point.x, obs_pos.point.y, 0, 1;
            VectorXd h_obs_ground = T_boat_to_ground * h_obs_boat;
            geometry_msgs::PointStamped obs_pos_g;
            obs_pos_g.header = bbox_msg->header;
            obs_pos_g.header.frame_id = "map";
            obs_pos_g.point.x = h_obs_ground[0];
            obs_pos_g.point.y = h_obs_ground[1];
            obs_pos_g.point.z = h_obs_ground[2];
            std::cout << "obs_pos_ground x: " << obs_pos_g.point.x << "  y: " << obs_pos_g.point.y << endl;
            obs_ground_pub.publish(obs_pos_g);
        }
    }
}

void wtst_cb(const sailboat_message::WTST_msgConstPtr &wtst_msg) {
//    std::cout << "wtst yaw: " << wtst_msg->Yaw << std::endl;
    Eigen::Affine3d r = create_rotation_matrix(0, 0, wtst_msg->Yaw / 57.3);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(wtst_msg->PosX, wtst_msg->PosY, 0)));
    T_boat_to_ground = (t * r).matrix();

    double roll = wtst_msg->Roll / 57.3;
    get_tf_boat_camera(roll);

//    Eigen::MatrixXd T_ground_to_boat(4, 4);
//    T_ground_to_boat = T_boat_to_ground.inverse();

    // reverse procedure
//    P_ground_to_img = P_boat_to_img * T_ground_to_boat;
//    M_Mat = P_ground_to_img.block(0, 0, 3, 3);
//    std::cout << "M_Mat: " << M_Mat << std::endl;
//    M_Mat_inv = M_Mat.inverse();
//    p_4 = P_ground_to_img.block(0, 3, 3, 1);
//    std::cout << "p_4: " << p_4 << std::endl;
//
//    C_tidle = -M_Mat_inv * p_4;

}

void img_cb(const sensor_msgs::ImageConstPtr &img_in) {
    std::cout << "===========in img callback=============" << std::endl;
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in, "bgr8")->image.copyTo(src_img);

    std::vector<Eigen::VectorXd> pts;
//    Eigen::VectorXd h_pt_boat(4);
////    h_pt_boat << 10, -3, 0, 1;
//    h_pt_boat << 5, -3, 0, 1;
//    pts.push_back(h_pt_boat);
////    h_pt_boat << 10, 3, 0, 1;
////    pts.push_back(h_pt_boat);
////    h_pt_boat << 6, 3, 0, 1;
////    pts.push_back(h_pt_boat);
////    h_pt_boat << 6, -3, 0, 1;
//    h_pt_boat << 20, -3, 0, 1;
//    pts.push_back(h_pt_boat);
    pts.push_back(p_boat_reverse);
    std::cout << "p_boat_reverse: " << p_boat_reverse << std::endl;

//    Eigen::VectorXd h_pt_ground(4);
//    h_pt_ground << -13, 6, 0, 1;
//    pts.push_back(h_pt_ground);

    for (int i = 0; i < pts.size(); i++) {
//        VectorXd h_pt_cam = T_boat_to_cam * pts[i];
//        std::cout << h_pt_cam << std::endl;

//        VectorXd h_pt_img = K * h_pt_cam;
        Eigen::VectorXd h_pt_img = P_boat_to_img * pts[i];
//        Eigen::VectorXd h_pt_img = P_ground_to_img * pts[i];

        h_pt_img[0] = h_pt_img[0] / h_pt_img[2];
        h_pt_img[1] = h_pt_img[1] / h_pt_img[2];
        h_pt_img[2] = h_pt_img[2] / h_pt_img[2];

        std::cout << "pt_img: " << h_pt_img << std::endl;
        cv::Point ppp((int) h_pt_img[0], (int) h_pt_img[1]);
        cv::circle(src_img, ppp, 2, cv::Scalar(0, 0, 255), 2);

    }
    cv::Point ppp((int) bbox_bottom[0], (int) bbox_bottom[1]);
    cv::circle(src_img, ppp, 2, cv::Scalar(0, 255, 0), 1);

    //reverse (10, -3)->(416, 485) (6, -3)->(214, 505)
//    pts.clear();
//    Eigen::Vector3d h_pt_img;
//    h_pt_img << 416, 485, 1;
//    pts.push_back(h_pt_img);
//    h_pt_img << 214, 505, 1;
//    pts.push_back(h_pt_img);
//
//    h_pt_img << 572, 451, 1;
//    pts.push_back(h_pt_img);
//    for (int i = 0; i < pts.size(); i++) {
//        Eigen::Vector4d p_boat_reverse = get_3d_point_from_pixel(pts[i]);
//        std::cout << "pt_reverse: " << p_boat_reverse << std::endl;
//    }


    cv::imshow("img_with_projected_pt", src_img);
    cv::waitKey(5);
    std::cout << "======  ==========" << std::endl;
}


void bbox_img_cb(const tld_msgs::BoundingBoxConstPtr &bbox_msg,
                 const sensor_msgs::ImageConstPtr &img_msg) {
//    bbox_cb(bbox_msg);
//    img_cb(img_msg);

    cv::Mat src_img;
    cv_bridge::toCvShare(img_msg, "bgr8")->image.copyTo(src_img);
    int x = bbox_msg->x;
    int y = bbox_msg->y;
    int height = bbox_msg->height;
    int width = bbox_msg->width;
    // x_center y_center center point of bbox
    int x_center = x + width / 2;
    int y_center = y + height / 2;
    int y_bottom = y + height;

    cv::Point ppp((int) x_center, (int) y_center);
    cv::circle(src_img, ppp, 2, cv::Scalar(0, 255, 0), 1);
//    cv::imshow("img_with_projected_pt", src_img);
//    cv::waitKey(5);

    sensor_msgs::ImagePtr img_msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                                           src_img).toImageMsg();
    pub_img.publish(img_msg_out);

}



int main(int argc, char **argv) {
//    get_camera_info();
//    get_tf_boat_camera();
    ros::init(argc, argv, "obs_position");
    ros::NodeHandle nh;

    obs_boat_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_boat_position", 2);
    obs_ground_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_ground_position", 2);

//    ros::Subscriber bbox_sub = nh.subscribe("/tld_tracked_object", 2, &bbox_cb);
    ros::Subscriber wtst_sub = nh.subscribe("/wtst", 2, &wtst_cb);

//    ros::Subscriber img_sub = nh.subscribe("/camera/image_undistorted_rotated", 2, &img_cb);
//    ros::Subscriber img_sub = nh.subscribe("/camera/image_undistorted", 2, &img_cb);
//    ros::Subscriber img_sub = nh.subscribe("/camera/image_raw", 2, &img_cb);
//

//    message_filters::Subscriber<sailboat_message::WTST_msg> wtst_sub(nh, "/wtst", 2);
    message_filters::Subscriber<tld_msgs::BoundingBox> bbox_sub(nh, "/tld_tracked_object", 2);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/image_undistorted", 2);
//
    typedef message_filters::sync_policies::ApproximateTime<tld_msgs::BoundingBox, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), bbox_sub, img_sub);
    sync.registerCallback(boost::bind(&bbox_img_cb, _1, _2));


    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("/camera/image_pt_reverse", 2);

    ros::spin();
    return 0;
}
