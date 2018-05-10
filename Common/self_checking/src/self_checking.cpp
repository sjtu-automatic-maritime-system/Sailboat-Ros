#include "self_checking.h"


void SailboatSelfChecking::onInit(){

    AHRSSubscriber = nh.subscribe("/ahrs", 10, &SailboatSelfChecking::AHRSSubscriberCB, this);

    WTSTSubscriber = nh.subscribe("/wtst", 10, &SailboatSelfChecking::WTSTSubscriberCB, this);

    ArduinoSubscriber = nh.subscribe("/arduino", 10,&SailboatSelfChecking::ArduinoSubscriberCB, this);

    CameraSubscriber = nh.subscribe("/camera", 10, &SailboatSelfChecking::DynamixelSubscriberCB, this);

    RadarSubscriber = nh.subscribe("/radar", 10, &SailboatSelfChecking::CameraSubscribeCB, this);

    DynamixelSubscriber = nh.subscribe("/motor_states/pan_tilt_port", 10,&SailboatSelfChecking::RadarSubscribeCB, this);

    MachSubscriber = nh.subscribe("/mach", 10,&SailboatSelfChecking::MachSubscribeCB, this);

    //DynamixelCtlClient = nh.serviceClient<wa_ros_msgs::SetGimbalCtl>("/drone/gimbal_control_srv");
    
    CheckResultClient = nh.serviceClient<sailboat_message::Self_Checking_srv>("/self_checking_arduino_srv");

    //OutTimeClient = nh.serviceClient<sailboat_message::Out_Time_srv>("/out_time_srv");

    OutTimePub = nh.advertise<sailboat_message::Out_Time_msg>("out_time",10);

    if (!init_checking){
        printf("new inits\n");
        checkAHRS_result = 2;
        checkWTST_result = 2;
        checkArduino_result = 2;
        checkCamera_result = 2;
        checkRadar_result = 2;
        checkDynamixel_result = 2;
        checkDisk_result = 2;
        timeout = 5;
        onSelfCheckingInit();
        onOutTimeInit();
        init_checking = true;
    }
}


void SailboatSelfChecking::AHRSSubscriberCB(const sailboat_message::Ahrs_msg::ConstPtr &msg){
    //ROS_INFO("get ahrsMsgNum, %d",ahrsMsgNum);
    ahrs_timestamp = msg->header.stamp;
    if (ahrsMsgNum == 0){
        ahrs_timestamp_last = ahrs_timestamp;
        ahrs_hz = 0;
        ahrs_time_sum = 0;
    }

    ros::Duration ahrs_duration = ahrs_timestamp - ahrs_timestamp_last;
    double ahrs_sec = ahrs_duration.toSec();
    if (ahrs_sec < 0){
        ahrs_sec = 0;
    }
    ahrs_time_sum += ahrs_sec; 
    if (ahrsMsgNum > 1 and ahrs_time_sum != 0){
        ahrs_hz = ahrsMsgNum / ahrs_time_sum;
    }
    ahrs_timestamp_last = ahrs_timestamp;
    ahrsMsgNum += 1;
}


void SailboatSelfChecking::WTSTSubscriberCB(const sailboat_message::WTST_msg::ConstPtr &msg){
    wtst_timestamp = msg->header.stamp;
    if (wtstMsgNum == 0){
        wtst_timestamp_last = wtst_timestamp;
        wtst_hz = 0;
        wtst_time_sum = 0;
    }

    ros::Duration wtst_duration = wtst_timestamp - wtst_timestamp_last;
    double wtst_sec = wtst_duration.toSec();
    if (wtst_sec < 0){
        wtst_sec = 0;
    }
    wtst_time_sum += wtst_sec; 
    if (wtstMsgNum > 1 and wtst_time_sum != 0){
        wtst_hz = wtstMsgNum / wtst_time_sum;
    }
    wtst_timestamp_last = wtst_timestamp;
    wtstMsgNum += 1;
}

void SailboatSelfChecking::ArduinoSubscriberCB(const sailboat_message::Arduino_msg::ConstPtr &msg){
    arduino_timestamp = msg->header.stamp;
    if (arduinoMsgNum == 0){
        arduino_timestamp_last = arduino_timestamp;
        arduino_hz = 0;
        arduino_time_sum = 0;
    }

    ros::Duration arduino_duration = arduino_timestamp - arduino_timestamp_last;
    double arduino_sec = arduino_duration.toSec();
    if (arduino_sec < 0){
        arduino_sec = 0;
    }
    arduino_time_sum += arduino_sec; 
    if (arduinoMsgNum > 1 and arduino_time_sum != 0){
        arduino_hz = arduinoMsgNum / arduino_time_sum;
    }
    arduino_timestamp_last = arduino_timestamp;
    arduinoMsgNum += 1;
}

void SailboatSelfChecking::DynamixelSubscriberCB(const dynamixel_msgs::MotorStateList::ConstPtr &msg){
    dynamixel_sail = 1;
    dynamixel_rudder = 1;
}

void SailboatSelfChecking::CameraSubscribeCB(const sensor_msgs::Image::ConstPtr &msg){
    camera_timestamp = msg->header.stamp;
    if (cameraMsgNum == 0){
        camera_timestamp_last = camera_timestamp;
        camera_hz = 0;
        camera_time_sum = 0;
    }

    ros::Duration camera_duration = camera_timestamp - camera_timestamp_last;
    double camera_sec = camera_duration.toSec();
    if (camera_sec < 0){
        camera_sec = 0;
    }
    camera_time_sum += camera_sec; 
    if (cameraMsgNum > 1 and camera_time_sum != 0){
        camera_hz = cameraMsgNum / camera_time_sum;
    }
    camera_timestamp_last = camera_timestamp;
    cameraMsgNum += 1;
}
void SailboatSelfChecking::RadarSubscribeCB(const geometry_msgs::PoseArray::ConstPtr &msg){
    radar_timestamp = msg->header.stamp;
    if (radarMsgNum == 0){
        radar_timestamp_last = radar_timestamp;
        radar_hz = 0;
        radar_time_sum = 0;
    }

    ros::Duration radar_duration = radar_timestamp - radar_timestamp_last;
    double radar_sec = radar_duration.toSec();
    if (radar_sec < 0){
        radar_sec = 0;
    }
    radar_time_sum += radar_sec; 
    if (radarMsgNum > 1 and radar_time_sum != 0){
        radar_hz = radarMsgNum / radar_time_sum;
    }
    radar_timestamp_last = radar_timestamp;
    radarMsgNum += 1;
}


void SailboatSelfChecking::MachSubscribeCB(const sailboat_message::Mach_msg::ConstPtr &msg){
    mach_timestamp = msg->header.stamp;
    mach_timestamp_last = mach_timestamp;
}


void SailboatSelfChecking::checkAHRS(){
    ROS_INFO("start checkahrs");
    int wait_time = 0;
    bool ahrs_init_suc = false;
    while (wait_time < 10){
        if (ahrsMsgNum >= 5){
            ahrs_init_suc = true;
            break;
        }
        wait_time += 1;
        usleep(1000000);
        ROS_INFO("ahrs not init");
    }
    
    
    if (ahrs_init_suc){
        ROS_INFO("ahrs start and cal ahrs hz");
        ahrsMsgNum = 0;
        usleep(5000000);
        ROS_INFO("FINAL ahrs_hz = %f",ahrs_hz);
        checkAHRS_param = ahrs_hz;
        if (std::fabs(ahrs_hz -10) < 2){
            checkAHRS_result = 1;
        }
        else{
            checkAHRS_result = 0;
        }
    }
    else{
        checkAHRS_param = 0;
        checkAHRS_result = 0;
    }
}


void SailboatSelfChecking::checkWTST(){
    ROS_INFO("start checkwtst");
    int wait_time = 0;
    bool wtst_init_suc = false;
    while (wait_time < 10){
        if (wtstMsgNum >= 5){
            wtst_init_suc = true;
            break;
        }
        wait_time += 1;
        usleep(1000000);
        ROS_INFO("wtst not init");
    }
    
    
    if (wtst_init_suc){
        ROS_INFO("wtst start and cal wtst hz");
        wtstMsgNum = 0;
        usleep(5000000);
        ROS_INFO("FINAL wtst_hz = %f",wtst_hz);
        checkWTST_param = wtst_hz;
        if (std::fabs(wtst_hz -10) < 2){
            checkWTST_result = 1;
        }
        else{
            checkWTST_result = 0;
        }
    }
    else{
        checkWTST_param = 0;
        checkWTST_result = 0;
    }
}


void SailboatSelfChecking::checkArduino(){
    ROS_INFO("start checkArduino");
    int wait_time = 0;
    bool arduino_init_suc = false;
    while (wait_time < 10){
        if (arduinoMsgNum >= 5){
            arduino_init_suc = true;
            break;
        }
        wait_time += 1;
        usleep(1000000);
        ROS_INFO("arduino not init");
    }
    
    
    if (arduino_init_suc){
        ROS_INFO("arduino start and cal arduino hz");
        arduinoMsgNum = 0;
        usleep(5000000);
        ROS_INFO("FINAL arduino_hz = %f",arduino_hz);
        checkArduino_param = arduino_hz;
        if (std::fabs(arduino_hz -10) < 2){
            checkArduino_result = 1;
        }
        else{
            checkArduino_result = 0;
        }
    }
    else{
        checkArduino_param = 0;
        checkArduino_result = 0;
    }
}

void SailboatSelfChecking::checkDynamixel(){
    checkDynamixel_param = 0;
    checkDynamixel_result = 0;
}

void SailboatSelfChecking::checkCamera(){
    ROS_INFO("start checkCamera");
    int wait_time = 0;
    bool camera_init_suc = false;
    while (wait_time < 10){
        if (cameraMsgNum >= 5){
            camera_init_suc = true;
            break;
        }
        wait_time += 1;
        usleep(1000000);
        ROS_INFO("camera not init");
    }
    
    
    if (camera_init_suc){
        ROS_INFO("camera start and cal camera hz");
        cameraMsgNum = 0;
        usleep(5000000);
        ROS_INFO("FINAL camera_hz = %f",camera_hz);
        checkCamera_param = camera_hz;
        if (std::fabs(camera_hz -10) < 2){
            checkCamera_result = 1;
        }
        else{
            checkCamera_result = 0;
        }
    }
    else{
        checkCamera_param = 0;
        checkCamera_result = 0;
    }
}
void SailboatSelfChecking::checkRader(){
    ROS_INFO("start checkRader");
    int wait_time = 0;
    bool radar_init_suc = false;
    while (wait_time < 10){
        if (radarMsgNum >= 5){
            radar_init_suc = true;
            break;
        }
        wait_time += 1;
        usleep(1000000);
        ROS_INFO("radar not init");
    }
    
    
    if (radar_init_suc){
        ROS_INFO("radar start and cal radar hz");
        radarMsgNum = 0;
        usleep(5000000);
        ROS_INFO("FINAL radar_hz = %f",radar_hz);
        checkRadar_param = radar_hz;
        if (std::fabs(radar_hz -10) < 2){
            checkRadar_result = 1;
        }
        else{
            checkRadar_result = 0;
        }
    }
    else{
        checkRadar_param = 0;
        checkRadar_result = 0;
    }
}


void SailboatSelfChecking::checkDisk(){
    ROS_INFO("start checkDisk");
    struct statfs disk_info;
    char *path ="/home/";
    int ret = 0;
    if (ret == statfs(path, &disk_info) == -1)
    {
        //fprintf(stderr, "Failed to get file disk infomation,\
        　　errno:%u, reason:%s\n", errno, strerror(errno));
        ROS_INFO("Failed to get file disk infomation");
        checkDisk_param = 0;
        checkDisk_result = 0;
        return;
    }
    long long total_size = disk_info.f_blocks * disk_info.f_bsize;
    long long available_size = disk_info.f_bavail * disk_info.f_bsize;
    long long free_size = disk_info.f_bfree * disk_info.f_bsize;
    // //输出每个块的长度，linux下内存块为4KB
    // printf("block size: %ld bytes\n", disk_info.f_bsize);
    // //输出块个数
    // printf("total data blocks: %ld \n", disk_info.f_blocks);
    // //输出path所在磁盘的大小
    // printf("total file disk size: %d MB\n",total_size >> 20);
    // //输出非root用户可以用的磁盘空间大小
    // printf("avaiable size: %d MB\n",available_size >> 20);
    // //输出硬盘的所有剩余空间
    // printf("free size: %d MB\n",free_size >> 20);
    // //输出磁盘上文件节点个数
    // printf("total file nodes: %ld\n", disk_info.f_files);
    // //输出可用文件节点个数
    // printf("free file nodes: %ld\n", disk_info.f_ffree);
    // //输出文件名最大长度
    // printf("maxinum length of file name: %ld\n", disk_info.f_namelen);

    checkDisk_param = (float)free_size / total_size;
    ROS_INFO("free dize size: %f ",checkDisk_param);
    if (checkDisk_param > 0.2){
        checkDisk_result = 1;
    }
    else{
        checkDisk_result = 0;
    }

}

void SailboatSelfChecking::sendresult(){
    
    ROS_INFO("checking param : ahrs = %f, wtst = %f, arduino = %f, camera = %f, radar = %f, dynamixel = %f, disk = %f",checkAHRS_param, checkWTST_param, checkArduino_param, checkCamera_param, checkRadar_param, checkDynamixel_param,checkDisk_param);
    ROS_INFO("checking result : ahrs = %d, wtst = %d, arduino = %d, camera = %d, radar = %d, dynamixel = %d, disk = %d",checkAHRS_result, checkWTST_result, checkArduino_result, checkCamera_result, checkRadar_result, checkDynamixel_result, checkDisk_result);

    while (!finish_checking){
        sailboat_message::Self_Checking_srv srv;
        srv.request.all_result = 1;
        srv.request.checkAHRS_param = checkAHRS_param;
        srv.request.checkWTST_param = checkWTST_param;
        srv.request.checkArduino_param = checkArduino_param;
        srv.request.checkCamera_param = checkCamera_param;
        srv.request.checkRadar_param = checkRadar_param;
        srv.request.checkDynamixel_param = checkDynamixel_param;
        srv.request.checkDisk_param = checkDisk_param;
        srv.request.checkAHRS_result = checkAHRS_result;
        srv.request.checkWTST_result = checkWTST_result;
        srv.request.checkArduino_result = checkArduino_result;
        srv.request.checkCamera_result = checkCamera_result;
        srv.request.checkRadar_result = checkRadar_result;
        srv.request.checkDynamixel_result = checkDynamixel_result;
        srv.request.checkDisk_result = checkDisk_result;

        if (CheckResultClient.call(srv))
        {
            ROS_INFO("checkResultClient result: %d", (int)srv.response.ack);
            finish_checking = true;
        }
        else
        {
            ROS_ERROR("Failed to call service checkResultClient");
        }
        usleep(1000000);
    }

    CameraSubscriber.shutdown();
    RadarSubscriber.shutdown();
    DynamixelSubscriber.shutdown();
}

void SailboatSelfChecking::stateUpdate(){

    ros::Rate loop_rate(1);
    while (ros::ok()){
        ros::Time time_now = ros::Time::now();
        // Enforce command timeout
        ros::Duration ahrs_time = time_now - ahrs_timestamp_last;
        double ahrs_dtime = ahrs_time.toSec();
        if ( ahrs_dtime > timeout ){
            AHRS_outTime = false;
            ROS_ERROR("ahrs Command timeout!");
        }
        else{
            AHRS_outTime = true;
        }
        ros::Duration wtst_time = time_now - wtst_timestamp_last;
        double wtst_dtime = wtst_time.toSec();
        if ( wtst_dtime > timeout ){
            WTST_outTime = false;
            ROS_ERROR("wtst Command timeout!");
        }
        else{
            WTST_outTime = true;
        }
        ros::Duration arduino_time = time_now - arduino_timestamp_last;
        double arduino_dtime = arduino_time.toSec();
        if ( arduino_dtime > timeout ){
            Arduino_outTime = false;
            ROS_ERROR("arduino Command timeout!");
        }
        else{
            Arduino_outTime = true;
        }
        ros::Duration mach_time = time_now - mach_timestamp_last;
        double mach_dtime = mach_time.toSec();
        if ( mach_dtime > timeout ){
            Mach_outTime = false;
            ROS_ERROR("mach Command timeout!");
        }
        else{
            Mach_outTime = true;
        }

        // if (!AHRS_outTime || !WTST_outTime || !Arduino_outTime || !Mach_outTime){
        //     sailboat_message::Out_Time_srv srv;
        //     srv.request.AHRS_outTime = AHRS_outTime;
        //     srv.request.WTST_outTime = WTST_outTime;
        //     srv.request.Arduino_outTime = Arduino_outTime;
        //     srv.request.Mach_outTime = Mach_outTime;

        //     if (OutTimeClient.call(srv))
        //     {
        //         ROS_INFO("checkResultClient result: %d", (int)srv.response.ack);
        //     }
        //     else
        //     {
        //         ROS_ERROR("Failed to call service checkResultClient");
        //     }
        // }
        sailboat_message::Out_Time_msg msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "out_time";
        msg.AHRS_outTime = AHRS_outTime;
        msg.WTST_outTime = WTST_outTime;
        msg.Arduino_outTime = Arduino_outTime;
        msg.Mach_outTime = Mach_outTime;
        OutTimePub.publish(msg);

        loop_rate.sleep();
    }
}


void SailboatSelfChecking::onSelfCheckingInit() {
    pthread_t thread_check;
    if (0 != pthread_create(&thread_check,NULL, SailboatSelfChecking::threadSelfChecking, this)){
        ROS_INFO("thread create failed");
    }
    else
        ROS_INFO("thread established");
}

void* SailboatSelfChecking::threadSelfChecking(void* args){
    SailboatSelfChecking::getInstance().checkAHRS();
    usleep(500000);
    SailboatSelfChecking::getInstance().checkWTST();
    usleep(500000);
    SailboatSelfChecking::getInstance().checkArduino();
    usleep(500000);
    SailboatSelfChecking::getInstance().checkDynamixel();
    usleep(500000);
    SailboatSelfChecking::getInstance().checkCamera();
    usleep(500000);
    SailboatSelfChecking::getInstance().checkRader();
    usleep(500000);
    SailboatSelfChecking::getInstance().checkDisk();
    usleep(500000);
    SailboatSelfChecking::getInstance().sendresult();
    ROS_INFO("SelfChecking finished !!!");
    usleep(1000000);
    //ros::shutdown();
}


void SailboatSelfChecking::onOutTimeInit() {
    pthread_t thread_check;
    if (0 != pthread_create(&thread_check,NULL, SailboatSelfChecking::threadOutTime, this)){
        ROS_INFO("thread create failed");
    }
    else
        ROS_INFO("thread established");
}

void* SailboatSelfChecking::threadOutTime(void* args){
    SailboatSelfChecking::getInstance().stateUpdate();
}


SailboatSelfChecking &SailboatSelfChecking::getInstance() {
    static SailboatSelfChecking *l_pAdp = nullptr;
    if (l_pAdp == nullptr)
    {
        l_pAdp = new SailboatSelfChecking();
        l_pAdp->onInit();
    }
    return *l_pAdp;
}
