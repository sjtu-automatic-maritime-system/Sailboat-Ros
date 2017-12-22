#include "vision_landing/track_targets.h"

// main..
int main(int argc, char** argv) {
    
    //ros init
    ros::init(argc, argv, "track_target");
    // Unbuffer stdout and stdin
    // cout.setf(ios::unitbuf);
    // ios_base::sync_with_stdio(false);

    TrackTarget track_target;

    // Setup arguments for parser
    // args::ArgumentParser parser("Track fiducial markers and estimate pose, output translation vectors for vision_landing");
    // args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    // args::Flag verbose(parser, "verbose", "Verbose", {'v', "verbose"});
    // args::ValueFlag<int> markerid(parser, "markerid", "Marker ID", {'i', "id"});
    // args::ValueFlag<string> dict(parser, "dict", "Marker Dictionary", {'d', "dict"});
    // args::ValueFlag<string> output(parser, "output", "Output Stream", {'o', "output"});
    // args::ValueFlag<int> width(parser, "width", "Video Input Resolution - Width", {'w', "width"});
    // args::ValueFlag<int> height(parser, "height", "Video Input Resolution - Height", {'g', "height"});
    // args::ValueFlag<int> fps(parser, "fps", "Video Output FPS - Kludge factor", {'f', "fps"});
    // args::ValueFlag<double> brightness(parser, "brightness", "Camera Brightness/Gain", {'b', "brightness"});
    // args::ValueFlag<string> sizemapping(parser, "sizemapping", "Marker Size Mappings, in marker_id:size format, comma separated", {'z', "sizemapping"});
    // args::ValueFlag<int> markerhistory(parser, "markerhistory", "Marker tracking history, in frames", {"markerhistory"});
    // args::ValueFlag<int> markerthreshold(parser, "markerthreshold", "Marker tracking threshold, percentage", {"markerthreshold"});
    // args::ValueFlag<string> fourcc(parser, "fourcc", "FourCC CoDec code", {'c', "fourcc"});
    // args::Positional<string> input(parser, "input", "Input Stream");
    // args::Positional<string> calibration(parser, "calibration", "Calibration Data");
    // args::Positional<double> markersize(parser, "markersize", "Marker Size");
    
    // Parse arguments    

    // Register signals
    // signal(SIGINT, handle_sig);
    // signal(SIGTERM, handle_sig);
    // signal(SIGUSR1, handle_sigusr1);
    // signal(SIGUSR2, handle_sigusr2);

    // Set camera properties
    // vreader.set(CAP_PROP_BRIGHTNESS, inputbrightness);
    // vreader.set(CV_CAP_PROP_FRAME_WIDTH, inputwidth);
    // vreader.set(CV_CAP_PROP_FRAME_HEIGHT, inputheight);
    // vreader.set(CV_CAP_PROP_FPS, inputfps);

    //get input rawimage
    // Take a single image and resize calibration parameters based on input stream dimensions
    //vreader >> rawimage;

    //output
    // Create an output object, if output specified then setup the pipeline unless output is set to 'window'
    // VideoWriter writer;
    // if (output && args::get(output) != "window") {
    //     if (fourcc) {
    //         string _fourcc = args::get(fourcc);
    //         writer.open(args::get(output), CV_FOURCC(_fourcc[0], _fourcc[1], _fourcc[2], _fourcc[3]), inputfps, cv::Size(inputwidth, inputheight), true);
    //     } else {
    //         writer.open(args::get(output), 0, inputfps, cv::Size(inputwidth, inputheight), true);
    //     }
    //     if (!writer.isOpened()) {
    //         cerr << "Error can't create video writer" << endl;
    //         return 1;
    //     }
    // }
    // Setup the marker detection
    
    ros::spin();
    
    return 0;
}


// Setup sig handling
// static volatile sig_atomic_t sigflag = 0;
// static volatile sig_atomic_t stateflag = 0; // 0 = stopped, 1 = started
// void handle_sig(int sig) {
//     cout << "info:SIGNAL:" << sig << ":Received" << endl;
//     sigflag = 1;
// }
// void handle_sigusr1(int sig) {
//     cout << "info:SIGNAL:SIGUSR1:Received:" << sig << ":Switching on Vision Processing" << endl;
//     stateflag = 1;
// }
// void handle_sigusr2(int sig) {
//     cout << "info:SIGNAL:SIGUSR2:Received:" << sig << ":Switching off Vision Processing" << endl;
//     stateflag = 0;
// }