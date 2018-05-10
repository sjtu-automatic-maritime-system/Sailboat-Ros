//
// Created by jianyun on 17-7-27.
//

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "object_detection_ros/detection.h"

int main() {
    cv::Mat image;

    //Load image
    std::string file_name = "/home/jianyun/catkin_ws/src/Sailboat-Ros/OBJECT_DETECTION/py_test/buoy/1.jpg";
    image = cv::imread(file_name, CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        std::cout << "could not open file" << std::endl;
        return -1;
    }

    //display image
    cv::namedWindow("raw", CV_WINDOW_AUTOSIZE);
    cv::imshow("raw", image);

//    cv::imwrite("result.jpg", image);
    cv::waitKey(0);

    cv::Mat edge = detection::edgeDetection(image);

    cv::imshow("edge", edge);

    cv::waitKey(0);

    std::vector<cv::Vec3f> circles = detection::circleDectection(image, edge);
    std::cout << "detected circles: " << circles.size() << std::endl;
    for (size_t i = 0; i < circles.size(); i++) {
        std::cout << "circle " << i << circles[i] << std::endl;
    }

    return 0;
}