#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "vision_landing/target.h"


 
using namespace std;
using namespace cv;
 
 
int main(int argc, char **argv) {
	VideoCapture inputVideo;
	int waitTime;
	inputVideo.open(0);
	inputVideo.set(CAP_PROP_FRAME_WIDTH, 1280);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, 720);
	waitTime = 10;
 
	double totalTime = 0;
	int totalIterations = 0;
 
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	Mat out;
	dictionary->drawMarker( 23, 200, out, 1);
 
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	detectorParams->minDistanceToBorder = 0;
 

	while (inputVideo.grab()) {
		Mat image, imageCopy;
		inputVideo.retrieve(image);
		double tick = (double)getTickCount();
		vector< int > ids;
		vector< vector< Point2f > > corners, rejected;
		vector< Vec3d > rvecs, tvecs;
		// detect markers and estimate pose
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
 
		double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
		totalTime += currentTime;
		totalIterations++;
		// draw results
		image.copyTo(imageCopy);
		if (ids.size() > 0) 
		{
			aruco::drawDetectedMarkers(imageCopy, corners, ids);
			if (totalIterations % 30 == 0) {
				cout << "Detection Time = " << currentTime * 1000 << " ms "
					<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
			}
		}
 
		imshow("out", imageCopy);
		char key = (char)waitKey(waitTime);
		if (key == 27) break;
	}
 
	return 0;
}
