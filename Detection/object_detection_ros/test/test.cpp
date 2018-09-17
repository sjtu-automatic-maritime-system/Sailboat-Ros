//
// Created by jianyun on 17-7-27.
//

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "object_detection_ros/detection.h"

int one(std::string file_name) {
    cv::Mat image;

    //Load image
    //std::string file_name = "/home/hywel/Pictures/1.png";
    image = cv::imread(file_name, CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        std::cout << "could not open file" << std::endl;
        return -1;
    }

    //display image
    cv::namedWindow("raw", CV_WINDOW_AUTOSIZE);
    cv::imshow("raw", image);


    cv::Mat edge = detection::edgeDetection(image);

    cv::imshow("edge", edge);


    std::vector<cv::Vec3f> circles = detection::circleDectection(image, edge);
    std::cout << "detected circles: " << circles.size() << std::endl;
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(image, center, radius, cv::Scalar(155, 50, 255), 3, 8, 0);
    }

    cv::imshow("hough circle", image);
    cv::waitKey(0);

    return 0;
}
int main()
{
    one("/home/hywel/Pictures/1.png");
    one("/home/hywel/Pictures/2.png");
    one("/home/hywel/Pictures/3.png");
    one("/home/hywel/Pictures/5.png");
    /* code */
    return 0;
}
