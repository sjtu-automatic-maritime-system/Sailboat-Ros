//#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

//#include <object_detection_ros/detection.h>
//using namespace detection;
namespace detection {
cv::Mat edgeDetection(cv::Mat img_src, u_int8_t low_threshold=100, u_int8_t high_threshold=200) {
//    cv::Mat img_clone = img_src.clone();
    cv::Mat gray, hsv;
    //imshow("src", img_src);

    int iLowH = 156;
    int iHighH = 180;
    int iLowS = 43;
    int iHighS = 255;
    int iLowV = 46;
    int iHighV = 255;

    cvtColor(img_src, gray, CV_BGR2GRAY);
    //imshow("edge0", edge);

    cvtColor(img_src, hsv, CV_BGR2HSV);
    
    //imshow("hsv", hsv);

    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);

    cv::Mat h_img = hsv_channels[0];
   // imshow("hue", h_img);

    cv::Mat s_img = hsv_channels[1];
    //imshow("saturation", s_img);

    cv::Mat v_img = hsv_channels[2];
    //imshow("v", v_img);
    
    cv::equalizeHist(hsv_channels[2], hsv_channels[2]);
    cv::merge(hsv_channels, hsv);

    //imshow("merge", hsv);
    
    cv::inRange(hsv, cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),gray);
    //edge = hls_channels[2];//mul(hls_channels[1]*2/255);
    //imshow("output", gray);

    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1, 1);

    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // cv::morphologyEx(gray, gray, cv::MORPH_OPEN, kernel);
    // cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, kernel);

    //imshow("morphologyEx", edge);

    //cv::Canny(edge, edge, 100, 200, 3);
    //imshow("Canny", edge);
    return gray;
}

std::vector<cv::Vec3f> circleDectection(cv::Mat &src_img, cv::Mat edge) {
    //cv::Mat gray;

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(edge, circles, CV_HOUGH_GRADIENT, 2, 100, 200, 100);

    if (circles.size()>10){
        std::vector<cv::Vec3f> circleNone;
        return circleNone;
    }

    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(src_img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(src_img, center, radius, cv::Scalar(155, 50, 255), 3, 8, 0);
    }
    //cv::imshow("hough circle", src_img);
    //cv::waitKey(0);
    return circles;
}
}//end of namespace
