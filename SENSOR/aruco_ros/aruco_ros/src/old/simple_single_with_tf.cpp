/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <queue>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <mavros_msgs/LandingTarget.h>

#define pi 3.1415926

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub; 
  ros::Publisher position_pub;
  ros::Publisher marker_pub; //rviz visualization marker
  ros::Publisher pixel_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  ros::ServiceClient client;

  double marker_size;
  int marker_id;
  uint32_t active_marker;

  map<uint32_t,float> markerSizes;
  map<uint32_t, queue<int> > marker_history_queue;
  //map<uint32_t,MarkerPoseTracker> MTracker;

  int marker_history;
  int marker_threshold;

  bool output;
  bool verbose;

  double fovx,fovy;
  int inputwidth,inputheight;


  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
  ArucoSimple()
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {

    std::string refinementMethod;
    nh.param<std::string>("corner_refinement", refinementMethod, "LINES");
    if ( refinementMethod == "SUBPIX" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
    else if ( refinementMethod == "HARRIS" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    else if ( refinementMethod == "NONE" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE); 
    else      
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES); 

    //Print parameters of aruco marker detector:
    ROS_INFO_STREAM("Corner refinement method: " << mDetector.getCornerRefinementMethod());
    ROS_INFO_STREAM("Threshold method: " << mDetector.getThresholdMethod());
    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: " << " th1: " << th1 << " th2: " << th2);
    float mins, maxs;
    mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << mDetector.getDesiredSpeed());
    
    active_marker = 0;

    marker_history = 30;
    marker_threshold = 50;

    output = true;
    verbose = false;

    image_sub = it.subscribe("/vision_landing_camera/image_raw", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/vision_landing_camera/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);

    client = nh.serviceClient<mavros_msgs::LandingTarget>("/mavros/set_landing_target");

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 0);
    nh.param<std::string>("reference_frame", reference_frame, "/base_link");
    nh.param<std::string>("camera_frame", camera_frame, "/vision_landing_camera_optical_frame");
    nh.param<std::string>("marker_frame", marker_frame, "/marker");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
             marker_size, marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }

  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(0.5),
                                       ros::Duration(0.01),
                                       &errMsg)
         )
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame,
                                     ros::Time(0),                  //get latest available
                                     transform);
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }


  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if ((image_pub.getNumSubscribers() == 0) &&
        (debug_pub.getNumSubscribers() == 0) &&
        (pose_pub.getNumSubscribers() == 0) &&
        (transform_pub.getNumSubscribers() == 0) &&
        (position_pub.getNumSubscribers() == 0) &&
        (marker_pub.getNumSubscribers() == 0) &&
        (pixel_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }

    //ROS_INFO("get image");

    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time curr_stamp(ros::Time::now());
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        //detection results will go into "markers"
        markers.clear();
        //Ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);

        map<float, uint32_t> markerAreas;
        map<uint32_t, bool> markerIds;

        for (size_t i=0; i<markers.size(); ++i) {
          //ROS_INFO("for marker");
          markerAreas[markers[i].getArea()] = markers[i].id;
          markerIds[markers[i].id] = true;

          if (marker_history_queue.count(markers[i].id) == 0) {
            for (unsigned int i=0; i<marker_history; i++) marker_history_queue[markers[i].id].push(0);
          }
          
        }

        // Iterate through marker history and update for this frame
        for( map <uint32_t, queue<int> >::iterator markerhist = marker_history_queue.begin();markerhist != marker_history_queue.end( );markerhist++)
        {
            //ROS_INFO("for marker history");
            // If marker was detected in this frame, push a 1
            (markerIds.count(markerhist->first)) ? markerhist->second.push(1) : markerhist->second.push(0);
            // If the marker history has reached history limit, pop the oldest element
            if (markerhist->second.size() > marker_history) {
                markerhist->second.pop();
            }
        }
        
        // If marker is set in config, use that to lock on
        if (marker_id) {
            //ROS_INFO("marker_id is setting");
            active_marker = marker_id;
        // Otherwise find the smallest marker that has a size mapping
        } else {
          //ROS_INFO("marker_id is't setting");
            for (map<float, uint32_t>::iterator markerArea = markerAreas.begin( ); markerArea != markerAreas.end( ); markerArea++ ) {
                uint32_t thisId = markerArea->second;
                if (markerSizes[thisId]) {
                    // If the current history for this marker is >threshold, then set as the active marker and clear marker histories.  Otherwise, skip to the next sized marker.
                    uint32_t _histsum = markerHistory(marker_history_queue, thisId, marker_history);
                    float _histthresh = marker_history * ((float)marker_threshold / (float)100);
                    if (_histsum > _histthresh) {
                        if (active_marker == thisId) break; // Don't change to the same thing
                        changeActiveMarker(marker_history_queue, active_marker, thisId, marker_history);
                        if (verbose) {
                            cout << "debug:changing active_marker:" << thisId << ":" << _histsum << ":" << _histthresh << ":" << endl;
                            cout << "debug:marker history:";
                            for (map <uint32_t, queue<int> >::iterator markerhist = marker_history_queue.begin( ); markerhist != marker_history_queue.end( ); markerhist++) {
                                cout << markerhist->first << ":" << markerHistory(marker_history_queue, markerhist->first, marker_history) << ":";
                            }
                            cout << endl;
                        }
                        break;
                    }
                }
            }
        }
        // If a marker lock hasn't been found by this point, use the smallest found marker with the default marker size
        if (!active_marker) {
          //ROS_INFO("active_marker is't setting");
            for (map<float, uint32_t>::iterator markerArea = markerAreas.begin( ); markerArea != markerAreas.end( ); markerArea++) {
                uint32_t thisId = markerArea->second;
                // If the history threshold for this marker is >50%, then set as the active marker and clear marker histories.  Otherwise, skip to the next sized marker.
                uint32_t _histsum = markerHistory(marker_history_queue, thisId, marker_history);
                float _histthresh = marker_history * ((float)marker_threshold / (float)100);
                //ROS_INFO("histsum~%d : histthresh~%f",_histsum,_histthresh);
                if (_histsum > _histthresh) {
                    cout << "debug:changing active_marker:" << thisId << endl;
                    changeActiveMarker(marker_history_queue, active_marker, thisId, marker_history); 
                    break;
                }
            }
        }

        // Iterate through the markers, in order of size, and do pose estimation
        for (map<float, uint32_t>::iterator markerArea = markerAreas.begin( ); markerArea != markerAreas.end( ); markerArea++) {
            if (markerArea->second != active_marker) continue; // Don't do pose estimation if not active marker, save cpu cycles
            float _size;
            // If marker size mapping exists for this marker, use it for pose estimation
            if (markerSizes[markerArea->second]) {
                _size = markerSizes[markerArea->second];
            // Otherwise use generic marker size
            } else if (marker_size) {
                _size = marker_size;
                cout << "debug:defaulting to generic marker size: " << markerArea->second << endl;
            }
            // Find the Marker in the Markers map and do pose estimation.  I'm sure there's a better way of iterating through the map..
            for (unsigned int i = 0; i < markers.size(); i++) {
                if (markers[i].id == markerArea->second)  {
                    //MTracker[markerArea->second].estimatePose(markers[i],camParam,_size);
                    float area = markers[i].getArea();
                    //ROS_INFO("marker[].id~%d : marker[].area~%f",markers[i].id,area);
                }
            }
        }

        // Iterate through each detected marker and send data for active marker and draw green AR cube, otherwise draw red AR square
        for (unsigned int i = 0; i < markers.size(); i++) {
            // If marker id matches current active marker, draw a green AR cube
            if (markers[i].id == active_marker) {
                if (output) {
                    markers[i].draw(inImage, cv::Scalar(0, 255, 0), 2, false);
                }
                // If pose estimation was successful, draw AR cube and distance
                if (markers[i].Tvec.at<float>(0,2) > 0) {
                    // Calculate vector norm for distance
                    double distance = sqrt(pow(markers[i].Tvec.at<float>(0,0), 2) + pow(markers[i].Tvec.at<float>(0,1), 2) + pow(markers[i].Tvec.at<float>(0,2), 2));
                    // Calculate angular offsets in radians of center of detected marker
                    double xoffset = (markers[i].getCenter().x - inputwidth / 2.0) * (fovx * (pi/180)) / inputwidth;
                    double yoffset = (markers[i].getCenter().y - inputheight / 2.0) * (fovy * (pi/180)) / inputheight;
                    if (verbose)
                    {
                        cout << "debug:active_marker:" << active_marker << ":center~" << markers[i].getCenter() << ":area~" << markers[i].getArea() << ":marker~" << markers[i] << ":vectorz~" << markers[i].Tvec.at<float>(0,2) << ":vectornorm~" << distance << endl;
                        cout << "target:" << markers[i].id << ":" << xoffset << ":" << yoffset << ":" << distance << endl;
                    }
                    if (output) { // don't burn cpu cycles if no output
                        drawARLandingCube(inImage, markers[i], camParam);
                        CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
                        drawVectors(inImage, cv::Scalar (0,255,0), 1, (i+1)*20, markers[i].id, xoffset, yoffset, distance, markers[i].getCenter().x, markers[i].getCenter().y);
                        
                        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
                        tf::StampedTransform cameraToReference;
                        cameraToReference.setIdentity();

                        if ( reference_frame != camera_frame )
                        {
                          getTransform(reference_frame,
                                      camera_frame,
                                      cameraToReference);
                        }

                        transform = 
                          static_cast<tf::Transform>(cameraToReference) 
                          * static_cast<tf::Transform>(rightToLeft) 
                          * transform;

                        // ROS_INFO("topic_pub");

                        tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                              reference_frame, marker_frame);
                        br.sendTransform(stampedTransform);
                        geometry_msgs::PoseStamped poseMsg;
                        tf::poseTFToMsg(transform, poseMsg.pose);
                        poseMsg.header.frame_id = reference_frame;
                        poseMsg.header.stamp = curr_stamp;
                        pose_pub.publish(poseMsg);
                        

                        geometry_msgs::TransformStamped transformMsg;
                        tf::transformStampedTFToMsg(stampedTransform, transformMsg);
                        transform_pub.publish(transformMsg);

                        geometry_msgs::Vector3Stamped positionMsg;
                        positionMsg.header = transformMsg.header;
                        positionMsg.vector = transformMsg.transform.translation;
                        position_pub.publish(positionMsg);

                        geometry_msgs::PointStamped pixelMsg;
                        pixelMsg.header = transformMsg.header;
                        pixelMsg.point.x = markers[i].getCenter().x;
                        pixelMsg.point.y = markers[i].getCenter().y;
                        pixelMsg.point.z = 0;
                        pixel_pub.publish(pixelMsg);

                        // //Publish rviz marker representing the ArUco marker patch
                        // visualization_msgs::Marker visMarker;
                        // visMarker.header = transformMsg.header;
                        // visMarker.pose = poseMsg.pose;
                        // visMarker.id = 1;
                        // visMarker.type   = visualization_msgs::Marker::CUBE;
                        // visMarker.action = visualization_msgs::Marker::ADD;
                        // visMarker.pose = poseMsg.pose;
                        // visMarker.scale.x = marker_size;
                        // visMarker.scale.y = 0.001;
                        // visMarker.scale.z = marker_size;
                        // visMarker.color.r = 1.0;
                        // visMarker.color.g = 0;
                        // visMarker.color.b = 0;
                        // visMarker.color.a = 1.0;
                        // visMarker.lifetime = ros::Duration(3.0);
                        // marker_pub.publish(visMarker);
                    }
                }
            // Otherwise draw a red square
            } else {
                if (output) { // don't burn cpu cycles if no output
                    markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2, false);
                    drawVectors(inImage, cv::Scalar (0,0,255), 1, (i+1)*20, markers[i].id, 0, 0, markers[i].Tvec.at<float>(0,2), markers[i].getCenter().x, markers[i].getCenter().y);
                }
            }
        }


        //for each marker, draw info and its boundaries in the image
        //for(size_t i=0; i<markers.size(); ++i)
        //{
          // only publishing the selected marker

          //float area = markers[i].getArea();
          //ROS_INFO("marker.size~%d : marker[].id~%d : marker_id~%d : marker[].area~%f",markers.size(),markers[i].id,marker_id,area);
          //if(false)
          //if(markers[i].id == marker_id)
          //{

        
          //}
          // but drawing all the detected markers
          //markers[i].draw(inImage,cv::Scalar(0,0,255),2);
        //}

        //draw a 3d cube in each marker if there is 3d info
        if(camParam.isValid() && marker_size!=-1)
        {
          for(size_t i=0; i<markers.size(); ++i)
          {
            //CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if(image_pub.getNumSubscribers() > 0)
        {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    inputheight = msg.height;
    inputwidth = msg.width;


    fovx = 2 * atan(inputwidth / (2 * camParam.CameraMatrix.at<float>(0,0))) * (180.0/3.1415926); 
    fovy = 2 * atan(inputheight / (2 * camParam.CameraMatrix.at<float>(1,1))) * (180.0/3.1415926);
    //ROS_INFO("info:FoVx~%f :FoVy~%f :vWidth~%d :vHeight~%d",fovx,fovy,inputwidth,inputheight);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }


  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setThresholdParams(config.param1,config.param2);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }

  void drawARLandingCube(cv::Mat &Image, Marker &m, const CameraParameters &CP) {
    cv::Mat objectPoints(8, 3, CV_32FC1);
    double halfSize = m.ssize / 2;
    objectPoints.at< float >(0, 0) = -halfSize;
    objectPoints.at< float >(0, 1) = -halfSize;
    objectPoints.at< float >(0, 2) = 0;
    objectPoints.at< float >(1, 0) = halfSize;
    objectPoints.at< float >(1, 1) = -halfSize;
    objectPoints.at< float >(1, 2) = 0;
    objectPoints.at< float >(2, 0) = halfSize;
    objectPoints.at< float >(2, 1) = halfSize;
    objectPoints.at< float >(2, 2) = 0;
    objectPoints.at< float >(3, 0) = -halfSize;
    objectPoints.at< float >(3, 1) = halfSize;
    objectPoints.at< float >(3, 2) = 0;

    objectPoints.at< float >(4, 0) = -halfSize;
    objectPoints.at< float >(4, 1) = -halfSize;
    objectPoints.at< float >(4, 2) = m.ssize;
    objectPoints.at< float >(5, 0) = halfSize;
    objectPoints.at< float >(5, 1) = -halfSize;
    objectPoints.at< float >(5, 2) = m.ssize;
    objectPoints.at< float >(6, 0) = halfSize;
    objectPoints.at< float >(6, 1) = halfSize;
    objectPoints.at< float >(6, 2) = m.ssize;
    objectPoints.at< float >(7, 0) = -halfSize;
    objectPoints.at< float >(7, 1) = halfSize;
    objectPoints.at< float >(7, 2) = m.ssize;

    vector< cv::Point2f > imagePoints;
    cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    // draw lines of different colours
    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 255, 0, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], cv::Scalar(0, 255, 0, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[i + 4], cv::Scalar(0, 255, 0, 255), 1, CV_AA);
  }

  // Print the calculated distance at bottom of image
  void drawVectors(cv::Mat &in, cv::Scalar color, int lineWidth, int vOffset, int MarkerId, double X, double Y, double Distance, double cX, double cY) {
      char cad[100];
      sprintf(cad, "ID:%i, Distance:%0.3fm, X:%0.3f, Y:%0.3f, cX:%0.3f, cY:%0.3f", MarkerId, Distance, X, Y, cX, cY);
      cv::Point cent(10, vOffset);
      cv::putText(in, cad, cent, cv::FONT_HERSHEY_SIMPLEX, std::max(0.5f,float(lineWidth)*0.3f), color, lineWidth);

      mavros_msgs::LandingTarget srv;
      srv.request.target_num = 2;
      srv.request.angle_x = X;
      srv.request.angle_y = Y;
      srv.request.size_x = 0;
      srv.request.size_y = 0;
      if (client.call(srv))
      {
        ROS_INFO("bool landing tartget:", srv.response.landing_target_sent);
      }
      else
      {
        ROS_ERROR("Failed to call service landing target");
      }

  }

  int markerHistory(map<uint32_t, queue<int> > &marker_history_queue, uint32_t thisId, uint32_t marker_history) {
    // Work out current history for this marker
    uint32_t histsum = 0;
    for (unsigned int j = 0; j < marker_history; j++) {
        uint32_t _val = marker_history_queue[thisId].front(); // fetch value off front of queue
        histsum += _val; // add value to history sum
        marker_history_queue[thisId].pop(); // pop value off front of queue
        marker_history_queue[thisId].push(_val); // push value back onto end of queue
    }
    return histsum;
  }

  void changeActiveMarker(map<uint32_t, queue<int> > &marker_history_queue, uint32_t &active_marker, uint32_t newId, uint32_t marker_history) {
    // Set the new marker as active
    active_marker = newId;

    // Reset all marker histories.  This ensures that another marker change can't happen for at least x frames where x is history size
    for ( map <uint32_t, queue<int> >::iterator markerhist = marker_history_queue.begin( ); markerhist != marker_history_queue.end( ); markerhist++ ) {
        for (unsigned int j = 0; j < marker_history; j++) {
            marker_history_queue[markerhist->first].push(0); marker_history_queue[markerhist->first].pop();
        }
    }
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
