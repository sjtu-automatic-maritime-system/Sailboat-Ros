#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace detection {
    cv::Mat edgeDetection(cv::Mat img_src, u_int8_t low_threshold = 100, u_int8_t high_threshold = 200);

    std::vector<cv::Vec3f> circleDectection(cv::Mat &src_img, cv::Mat edge);
}





