//#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
//#include <object_detection_ros/detection.h>
//using namespace detection;
namespace detection {
cv::Mat edgeDetection(cv::Mat img_src, u_int8_t low_threshold=100, u_int8_t high_threshold=200) {
//    cv::Mat img_clone = img_src.clone();
    cv::Mat dst, edge, gray, hls;

    dst.create(img_src.size(), img_src.type());

    cvtColor(img_src, gray, CV_BGR2GRAY);

    cvtColor(img_src, hls, CV_BGR2HLS);
    std::vector<cv::Mat> hls_channels;
    cv::split(hls, hls_channels);
    cv::Mat h_img = hls_channels[0];
    //imshow("hue", h_img);
    //cv::waitKey(5);
    cv::Mat l_img = hls_channels[1];
    //imshow("lightness", l_img);
    //cv::waitKey(0);
    cv::Mat s_img = hls_channels[2];
    //imshow("saturation", s_img);
    cv::waitKey(5);

    cv::GaussianBlur(s_img, edge, cv::Size(9, 9), 2, 2);

    cv::Canny(edge, edge, low_threshold, high_threshold, 3);

    dst = cv::Scalar::all(0);

    img_src.copyTo(dst, edge);

    //imshow("dst", dst);
    //cv::waitKey(0);

    return edge;
}


std::vector<cv::Vec3f> circleDectection(cv::Mat &src_img, cv::Mat edge) {
    //cv::Mat gray;

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(edge, circles, CV_HOUGH_GRADIENT, 2, 50, 200, 100, 0, 100);

    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(src_img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(src_img, center, radius, cv::Scalar(155, 50, 255), 3, 8, 0);
    }
    //cv::imshow("hough circle", edge);
    //cv::waitKey(0);
    return circles;
}
}//end of namespace
