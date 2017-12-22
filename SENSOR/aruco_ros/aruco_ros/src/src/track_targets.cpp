/**
track_targets
https://github.com/fnoop/vision_landing

This program uses opencv and aruco to detect fiducial markers in a stream.
It uses tracking across images in order to avoid ambiguity problem with a single marker.
For each target detected, it performs pose estimation and outputs the marker ID and translation vectors.
These vectors are used by vision_landing mavlink messages to enable precision landing by vision alone.

Compile with cmake: cmake . && make
 or manually: g++ src/track_targets.cpp -o track_targets -std=gnu++11 `pkg-config --cflags --libs aruco`

Run separately with: ./track_targets -d TAG36h11 /dev/video0 calibration.yml 0.235
./track_targets -w 1280 -g 720 -d TAG36h11 -o 'appsrc ! autovideoconvert ! v4l2video11h264enc extra-controls="encode,h264_level=10,h264_profile=4,frame_level_rate_control_enable=1,video_bitrate=2097152" ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false' /dev/video2 calibration/ocam5cr-calibration-1280x720.yml 0.235
./track_targets -w 1280 -g 720 -d TAG36h11 -o /srv/maverick/data/videos/landing.avi /dev/video2 calibration/ocam5cr-calibration-1280x720.yml 0.235
./track_targets -b 0.8 -d TAG36h11 -o 'appsrc ! autovideoconvert ! v4l2video11h264enc extra-contros="encode,h264_level=10,h264_profile=4,frame_level_rate_control_enable=1,video_bitrate=2097152" ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false' -i 1 -v /dev/video2 calibration/ocam5cr-calibration-640x480.yml 0.235
**/

///home/hywel/aruco/src/markerlabelers/dictionary_based.cpp

#include "vision_landing/track_targets.h"

TrackTarget::TrackTarget(): it_(nh_){
    init();
}
TrackTarget::~TrackTarget(){
}

void TrackTarget::init(){

    ROS_INFO("start init");
    
    input_image_topic = "/camera/image_input/image_raw";
    output_image_topic = "/camera/image_output";
    
    calibration_file = "/home/hywel/vision_landing/calibration/ocam5cr-calibration-640x480.yml";

    inputwidth = 640;
    inputheight = 480;
    inputfps = 30;
    inputbrightness = 0.5;

    markerid=12;

    output = true;
    imageresize = false;
    verbose = true;

    marker_history = 30;
    marker_threshold = 80;

    _avgdur=0;
    _fpsstart=0;
    _avgfps=0;
    _fps1sec=0;

    MarkerSize = 0.5;

    try{
    nh_.getParam("input_image_topic",input_image_topic);
    nh_.getParam("output_image_topic",output_image_topic);
    nh_.getParam("inputwidth",inputwidth);
    nh_.getParam("inputheight",inputheight);
    nh_.getParam("inputfps",inputfps);
    nh_.getParam("inputbrightness",inputbrightness);
    nh_.getParam("markerid",markerid);
    nh_.getParam("MarkerSize",MarkerSize);

    nh_.getParam("calibration_file",calibration_file);
    nh_.getParam("output", output);
    nh_.getParam("imageresize", imageresize);
    nh_.getParam("verbose", verbose);
    nh_.getParam("marker_history",marker_history);
    nh_.getParam("marker_threshold",marker_threshold);

    ROS_INFO("Successfully Loaded tracking parameters");
    }

    catch(int e)
    {
        ROS_WARN("Parameters are not properly loaded from file, loading defaults");
    }

    image_sub_ = it_.subscribe(input_image_topic, 1, &TrackTarget::imageCb,this);
    image_pub_ = it_.advertise(output_image_topic, 1);
   
    target_pub = nh_.advertise<vision_landing::target>("/vision_landing/target",10);

    // Read and parse camera calibration data
    CamParam.readFromXMLFile(calibration_file);
    if (!CamParam.isValid()) {
        ROS_INFO("Calibration Parameters not valid");
        return;
    }
    // Calculate the fov from the calibration intrinsics
    pi = std::atan(1)*4;

    fovx = 2 * atan(inputwidth / (2 * CamParam.CameraMatrix.at<float>(0,0))) * (180.0/pi); 
    fovy = 2 * atan(inputheight / (2 * CamParam.CameraMatrix.at<float>(1,1))) * (180.0/pi);
    ROS_INFO("info:FoVx~%f :FoVy~%f :vWidth~%d :vHeight~%d",fovx,fovy,inputwidth,inputheight);

    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);

    // if (dict)
    //     MDetector.setDictionary(args::get(dict), 0.f);

    // Start framecounter at 0 for fps tracking
    frameno=0;

    
    // Create a map of marker sizes from 'sizemapping' config setting
    
    //ss = NULL;
    // First split each mapping into a vector
    ss << "11:0.446,16:0.204,19:0.104,23:0.047";
    while(std::getline(ss, _size_mapping, ',')) {
        _size_mappings.push_back( _size_mapping );
    }
    // Next tokenize each vector element into the markerSizes map
    for (std::string const & _s : _size_mappings) {
        auto _i = _s.find(':');
        markerSizes[atoi(_s.substr(0,_i).data())] = atof(_s.substr(_i+1).data());
    }
    // Debug print the constructed map
    ROS_INFO("info:Size Mappings:");
    for (const auto &p : markerSizes) {
        //ROS_INFO("first~%f",p.first);
        cout << p.first << "=" << p.second << ", ";
    }
    cout << endl;

    // If marker history or threshold is set in parameters use them, otherwise set defaults
    if (marker_history) {
        marker_history = marker_history;
    } else {
        marker_history = 15;
    }
    ROS_INFO("debug:Marker History:%d",marker_history);
    //cout << "debug:Marker History:" << marker_history << endl;
    
    //marker_threshold = args::get(markerthreshold);
    ROS_INFO("debug:Marker Threshold:%d",marker_threshold);
    //cout << "debug:Marker Threshold:" << marker_threshold << endl;
    
    // Print a specific info message to signify end of initialisation
    ROS_INFO("info:initcomp:Initialisation Complete");
    //cout << "info:initcomp:Initialisation Complete" << endl;
}


void TrackTarget::imageCb(const sensor_msgs::ImageConstPtr& msg){
    namespace enc = sensor_msgs::image_encodings;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        ROS_INFO("get image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //rawimage = cv_ptr->image;

    if (!imageresize) {
        ROS_INFO("image resize");
        CamParam.resize(cv_ptr->image.size());
        imageresize = true;
    }
    main_loop(cv_ptr->image);
    image_pub_.publish(cv_ptr->toImageMsg());
}

void TrackTarget::main_loop(Mat& rawimage){
        
    // Lodge clock for start of frame
    //double framestart=CLOCK();
    
    //if (rawimage.empty()) return;

    ROS_INFO("track start");
    // Detect markers
    vector< Marker > Markers=MDetector.detect(rawimage,CamParam,MarkerSize);

    // Order the markers in ascending size - we want to start with the smallest.
    map<float, uint32_t> markerAreas;
    map<uint32_t, bool> markerIds;

    for (auto & marker:Markers) {
        ROS_INFO("for marker");
        markerAreas[marker.getArea()] = marker.id;
        markerIds[marker.id] = true;
        // If the marker doesn't already exist in the threshold tracking, add and populate with full set of zeros
        if (marker_history_queue.count(marker.id) == 0) {
            for (unsigned int i=0; i<marker_history; i++) marker_history_queue[marker.id].push(0);
        }
    }

    // Iterate through marker history and update for this frame
    for (auto & markerhist:marker_history_queue) {
        ROS_INFO("for marker history");
        // If marker was detected in this frame, push a 1
        (markerIds.count(markerhist.first)) ? markerhist.second.push(1) : markerhist.second.push(0);
        // If the marker history has reached history limit, pop the oldest element
        if (markerhist.second.size() > marker_history) {
            markerhist.second.pop();
        }
    }
    
    // If marker is set in config, use that to lock on
    if (markerid) {
        active_marker = markerid;
    // Otherwise find the smallest marker that has a size mapping
    } else {
        for (auto & markerArea:markerAreas) {
            uint32_t thisId = markerArea.second;
            if (markerSizes[thisId]) {
                // If the current history for this marker is >threshold, then set as the active marker and clear marker histories.  Otherwise, skip to the next sized marker.
                uint32_t _histsum = markerHistory(marker_history_queue, thisId, marker_history);
                float _histthresh = marker_history * ((float)marker_threshold / (float)100);
                if (_histsum > _histthresh) {
                    if (active_marker == thisId) break; // Don't change to the same thing
                    cout << "debug:changing active_marker:" << thisId << ":" << _histsum << ":" << _histthresh << ":" << endl;
                    changeActiveMarker(marker_history_queue, active_marker, thisId, marker_history);
                    if (verbose) {
                        cout << "debug:marker history:";
                        for (auto & markerhist:marker_history_queue) {
                            cout << markerhist.first << ":" << markerHistory(marker_history_queue, markerhist.first, marker_history) << ":";
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
        for (auto & markerArea:markerAreas) {
            uint32_t thisId = markerArea.second;
            // If the history threshold for this marker is >50%, then set as the active marker and clear marker histories.  Otherwise, skip to the next sized marker.
            uint32_t _histsum = markerHistory(marker_history_queue, thisId, marker_history);
            float _histthresh = marker_history * ((float)marker_threshold / (float)100);
            if (_histsum > _histthresh) {
                cout << "debug:changing active_marker:" << thisId << endl;
                changeActiveMarker(marker_history_queue, active_marker, thisId, marker_history); 
                break;
            }
        }
    }

    // Iterate through the markers, in order of size, and do pose estimation
    for (auto & markerArea:markerAreas) {
        if (markerArea.second != active_marker) continue; // Don't do pose estimation if not active marker, save cpu cycles
        float _size;
        // If marker size mapping exists for this marker, use it for pose estimation
        if (markerSizes[markerArea.second]) {
            _size = markerSizes[markerArea.second];
        // Otherwise use generic marker size
        } else if (MarkerSize) {
            _size = MarkerSize;
            cout << "debug:defaulting to generic marker size: " << markerArea.second << endl;
        }
        // Find the Marker in the Markers map and do pose estimation.  I'm sure there's a better way of iterating through the map..
        for (unsigned int i = 0; i < Markers.size(); i++) {
            if (Markers[i].id == markerArea.second)  {
                MTracker[markerArea.second].estimatePose(Markers[i],CamParam,_size);
            }
        }
    }

    // Iterate through each detected marker and send data for active marker and draw green AR cube, otherwise draw red AR square
    for (unsigned int i = 0; i < Markers.size(); i++) {
        // If marker id matches current active marker, draw a green AR cube
        if (Markers[i].id == active_marker) {
            if (output) {
                Markers[i].draw(rawimage, Scalar(0, 255, 0), 2, false);
            }
            // If pose estimation was successful, draw AR cube and distance
            if (Markers[i].Tvec.at<float>(0,2) > 0) {
                // Calculate vector norm for distance
                double distance = sqrt(pow(Markers[i].Tvec.at<float>(0,0), 2) + pow(Markers[i].Tvec.at<float>(0,1), 2) + pow(Markers[i].Tvec.at<float>(0,2), 2));
                // Calculate angular offsets in radians of center of detected marker
                double xoffset = (Markers[i].getCenter().x - inputwidth / 2.0) * (fovx * (pi/180)) / inputwidth;
                double yoffset = (Markers[i].getCenter().y - inputheight / 2.0) * (fovy * (pi/180)) / inputheight;
                if (verbose)
                    cout << "debug:active_marker:" << active_marker << ":center~" << Markers[i].getCenter() << ":area~" << Markers[i].getArea() << ":marker~" << Markers[i] << ":vectorz~" << Markers[i].Tvec.at<float>(0,2) << ":vectornorm~" << distance << endl;
                cout << "target:" << Markers[i].id << ":" << xoffset << ":" << yoffset << ":" << distance << endl;
                if (output) { // don't burn cpu cycles if no output
                    drawARLandingCube(rawimage, Markers[i], CamParam);
                    CvDrawingUtils::draw3dAxis(rawimage, Markers[i], CamParam);
                    drawVectors(rawimage, Scalar (0,255,0), 1, (i+1)*20, Markers[i].id, xoffset, yoffset, distance, Markers[i].getCenter().x, Markers[i].getCenter().y);
                }
            }
        // Otherwise draw a red square
        } else {
            if (output) { // don't burn cpu cycles if no output
                Markers[i].draw(rawimage, Scalar(0, 0, 255), 2, false);
                drawVectors(rawimage, Scalar (0,0,255), 1, (i+1)*20, Markers[i].id, 0, 0, Markers[i].Tvec.at<float>(0,2), Markers[i].getCenter().x, Markers[i].getCenter().y);
            }
        }
    }

    // if (output && args::get(output) != "window") {
    //     writer << rawimage;
    // } else if (output && args::get(output) == "window") {
    //     imshow("vision_landing", rawimage);
    // }

    //output

    // Lodge clock for end of frame   
    //double framedur = CLOCK()-framestart;
    // Print fps info every 100 frames if in debug mode
    //char framestr[100];
    //sprintf(framestr, "debug:avgframedur~%fms:fps~%f:frameno~%d:",avgdur(framedur),avgfps(),frameno++ );
    //if (verbose && (frameno % 100 == 1))
    //    cout << framestr << endl;
}

// Setup fps tracker
double TrackTarget::CLOCK()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,  &t);
    return (t.tv_sec * 1000)+(t.tv_nsec*1e-6);
}

double TrackTarget::avgdur(double newdur)
{
    _avgdur=0.98*_avgdur+0.02*newdur;
    return _avgdur;
}
double TrackTarget::avgfps()
{
    if(CLOCK()-_fpsstart>1000)      
    {
        _fpsstart=CLOCK();
        _avgfps=0.7*_avgfps+0.3*_fps1sec;
        _fps1sec=0;
    }
    _fps1sec++;
    return _avgfps;
}

// Define function to draw AR landing marker
void TrackTarget::drawARLandingCube(cv::Mat &Image, Marker &m, const CameraParameters &CP) {
    Mat objectPoints(8, 3, CV_32FC1);
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

    vector< Point2f > imagePoints;
    cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    // draw lines of different colours
    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], Scalar(0, 255, 0, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], Scalar(0, 255, 0, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[i + 4], Scalar(0, 255, 0, 255), 1, CV_AA);
}

// Print the calculated distance at bottom of image
void TrackTarget::drawVectors(Mat &in, Scalar color, int lineWidth, int vOffset, int MarkerId, double X, double Y, double Distance, double cX, double cY) {
    char cad[100];
    sprintf(cad, "ID:%i, Distance:%0.3fm, X:%0.3f, Y:%0.3f, cX:%0.3f, cY:%0.3f", MarkerId, Distance, X, Y, cX, cY);
    Point cent(10, vOffset);
    cv::putText(in, cad, cent, FONT_HERSHEY_SIMPLEX, std::max(0.5f,float(lineWidth)*0.3f), color, lineWidth);
}

// Summarise a marker history over the past 10 frames
int TrackTarget::markerHistory(map<uint32_t, queue<int>> &marker_history_queue, uint32_t thisId, uint32_t marker_history) {
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

// Change active marker and reset all marker histories
void TrackTarget::changeActiveMarker(map<uint32_t, queue<int>> &marker_history_queue, uint32_t &active_marker, uint32_t newId, uint32_t marker_history) {
    // Set the new marker as active
    active_marker = newId;

    // Reset all marker histories.  This ensures that another marker change can't happen for at least x frames where x is history size
    for (auto & markerhist:marker_history_queue) {
        for (unsigned int j = 0; j < marker_history; j++) {
            marker_history_queue[markerhist.first].push(0); marker_history_queue[markerhist.first].pop();
        }
    }
}






