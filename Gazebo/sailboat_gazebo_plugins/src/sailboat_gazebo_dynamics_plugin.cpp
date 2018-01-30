
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>

#include <sailboat_gazebo_plugins/sailboat_gazebo_dynamics_plugin.h>

#define GRAVITY 9.815

using namespace gazebo;

SailboatPlugin::SailboatPlugin()
{
}

SailboatPlugin::~SailboatPlugin()
{
    rosnode_->shutdown();
    spinner_thread_->join();
    delete rosnode_;
    delete spinner_thread_;
}

void SailboatPlugin::FiniChild()
{
}


double SailboatPlugin::getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val)
{
    double val = default_val;
    if (sdfPtr->HasElement(param_name) && sdfPtr->GetElement(param_name)->GetValue())
    {
        val = sdfPtr->GetElement(param_name)->Get<double>();
        ROS_INFO_STREAM("Parameter found - setting <" << param_name << "> to <" << val << ">.");

    }
    else{
        ROS_INFO_STREAM("Parameter <" << param_name << "> not found: Using default value of <" << val << ">.");
    }
    return val;
}

void SailboatPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    ROS_INFO("Loading sailboat_gazebo_dynamics_plugin");
    model_ = _parent;
    world_ = model_->GetWorld();

    // Retrieve model paramters from SDF
    // Set default values
    node_namespace_ = "";

    water_level_ = 0.5;
    water_density_ = 997.7735;
    cmd_timeout_ = 1.0;

    g=9.81;
    m = 15.3;
    Asm = 0.59;


    //  Enumerating model
    ROS_INFO_STREAM("Enumerating Model...");
    ROS_INFO_STREAM("Model name = "<< model_->GetName());
    physics::Link_V links = model_->GetLinks();
    for (int ii=0; ii<links.size(); ii++){
        ROS_INFO_STREAM("Link: "<<links[ii]->GetName());
    }

    // Get parameters from SDF
    if (_sdf->HasElement("robotNamespace"))
    {
        node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    if (!_sdf->HasElement("bodyName") || !_sdf->GetElement("bodyName")->GetValue())
    {
        link_ = model_->GetLink();
        link_name_ = link_->GetName();
        ROS_INFO_STREAM("Did not find SDF parameter bodyName");
    }
    else {
        link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
        link_ = model_->GetLink(link_name_);

        ROS_INFO_STREAM("Found SDF parameter bodyName as <"<<link_name_<<">");
    }
    if (!link_)
    {
        ROS_FATAL("sailboat_gazebo_dynamics_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
        return;
    }
    else
    {
        ROS_INFO_STREAM("Sailboat Model Link Name = " << link_name_);
    }

    water_level_ = getSdfParamDouble(_sdf,"waterLevel",water_level_);
    water_density_ = getSdfParamDouble(_sdf,"waterDensity",water_density_);
    cmd_timeout_ = getSdfParamDouble(_sdf,"cmdTimeout",cmd_timeout_);

    // Get inertia and mass of vessel
    math::Vector3 inertia = link_->GetInertial()->GetPrincipalMoments();
    double mass = link_->GetInertial()->GetMass();

    // Report some of the pertinent parameters for verification
    ROS_INFO("Sailboat Dynamics Parameters: From URDF XACRO model definition");
    ROS_INFO_STREAM("Vessel Mass (rigid-body): " << mass);
    ROS_INFO_STREAM("Vessel Inertia Vector (rigid-body): X:" << inertia[0] <<
                                                             " Y:"<<inertia[1] <<
                                                             " Z:"<<inertia[2]);
    ROS_INFO("Sailboat Dynamics Parameters: Plugin Parameters");

    //initialize time and odometry position
    prev_update_time_ = last_mach_drive_time_ = this->world_->GetSimTime();

    // Initialize the ROS node and subscribe to mach_drive
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "sailboat_gazebo",
              ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    rosnode_ = new ros::NodeHandle( node_namespace_ );

    mach_drive_sub_ = rosnode_->subscribe("mach", 1, &SailboatPlugin::OnMachDrive, this );
    wind_drive_sub_ = rosnode_->subscribe("wind", 1, &SailboatPlugin::OnWindDrive, this );

    ROS_INFO("ros init");

    //init
    //显示仿真数据
    //SME.ShowData();
    //set
    //SME.SettingAttitudeInit(0,0,0,0,0,0,0,pi);

    ROS_INFO("SME init");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->spinner_thread_ = new boost::thread( boost::bind( &SailboatPlugin::spin, this) );
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SailboatPlugin::UpdateChild, this));

    ROS_INFO("thread init");

    Ma_ = Eigen::MatrixXd(6,6);
    Ma_ << 5 ,   0,   0, 0, 0, 0,
            0,   5,   0, 0, 0, 0,
            0,   0,   0.1, 0, 0, 0,
            0,   0,   0, 0.1, 0, 0,
            0,   0,   0, 0, 0.1, 0,
            0,   0,   0, 0, 0, 1 ;
}


double SailboatPlugin::scaleThrustCmd(double cmd, double max_cmd, double max_pos, double max_neg)
{
    double val = 0.0;
    if (cmd >= 0.0){
        val = cmd/max_cmd*max_pos;
        if (val > max_pos)
        {
            val = max_pos;
        }
    }
    else  // cmd is less than zero
    {
        val = -1.0*std::abs(cmd)/max_cmd*std::abs(max_neg);
        if (std::abs(val) > std::abs(max_neg))
        {
            val = -1.0*std::abs(max_neg);  // ensure it is negative
        }
    }
    return val;
}
void SailboatPlugin::UpdateChild()
{

    common::Time time_now = this->world_->GetSimTime();
    common::Time step_time = time_now - prev_update_time_;

    double dt = step_time.Double();
    prev_update_time_ = time_now;

    // Enforce command timeout
    common::Time cmd_time = time_now - last_mach_drive_time_;
    double dcmd = cmd_time.Double();
    if ( dcmd > cmd_timeout_ )
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"Command timeout!");
        last_mach_drive_sail_ = 0.0;
        last_mach_drive_rudder_ = 0.0;
    }
    // Scale commands to thrust and torque forces
    //ROS_DEBUG_STREAM_THROTTLE(1.0,"Last mach: sail:" << last_mach_drive_sail_
    //                                                << " rudder: " << last_mach_drive_rudder_);

    // Get Pose/Orientation from Gazebo (if no state subscriber is active)
    pose = link_->GetWorldPose();
    euler = pose.rot.GetAsEuler();
    math::Quaternion curr_orientation = pose.rot;

    // Get body-centered linear and angular rates
    math::Vector3 vel_linear_body = link_->GetRelativeLinearVel();
    //ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel linear: " << vel_linear_body);
    math::Vector3 vel_angular_body = link_->GetRelativeAngularVel();
    //ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel angular: " << vel_angular_body);
    // Estimate the linear and angular accelerations.
    // Note the the GetRelativeLinearAccel() and AngularAccel() functions
    // appear to be unreliable
    math::Vector3 accel_linear_body = (vel_linear_body - prev_lin_vel_) /dt;
    prev_lin_vel_ = vel_linear_body;
    //ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel linear: " << accel_linear_body);
    math::Vector3 accel_angular_body = (vel_angular_body - prev_ang_vel_) /dt;
    prev_ang_vel_ = vel_angular_body;
    //ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel angular: " << accel_angular_body);

    // Create state and derivative of state (accelerations)
    Eigen::VectorXd state_dot(6);
    state_dot << accel_linear_body.x, accel_linear_body.y, accel_linear_body.z,
            accel_angular_body.x, accel_angular_body.y, accel_angular_body.z;

    Eigen::VectorXd state(6);
    state << vel_linear_body.x, vel_linear_body.y, vel_linear_body.z,
            vel_angular_body.x, vel_angular_body.y, vel_angular_body.z;

    // Added Mass
    Eigen::VectorXd amassVec = -1.0*Ma_*state_dot;
    ROS_DEBUG_STREAM_THROTTLE(1.0,"state_dot: \n" << state_dot);
    ROS_DEBUG_STREAM_THROTTLE(1.0,"amassVec :\n" << amassVec);

    //ROS_INFO("get pose");


    SME.nu(0) = vel_linear_body[0];
    SME.nu(1) = vel_linear_body[1];
    SME.nu(2) = vel_angular_body[0];
    SME.nu(3) = vel_angular_body[2];
    SME.eta(0) = pose.pos.x;
    SME.eta(1) = pose.pos.y;
    SME.eta(2) = euler[0];
    SME.eta(3) = euler[2];
    //ROS_INFO("pose = %f , %f",SME.eta(0),SME.eta(1));


    SME.Sailboat_Calc(dt);

    //ROS_INFO("calc");

    // Drag
    //Eigen::MatrixXd Dmat = Eigen::MatrixXd(6,6);
    Eigen::MatrixXd Dmat = Eigen::MatrixXd::Zero(6,6);
    Dmat(0,0) = 20 + 0*std::abs(vel_linear_body.x);
    Dmat(1,1) = 20 + 0*std::abs(vel_linear_body.y);
    Dmat(2,2) = 50;
    Dmat(3,3) = 10;
    Dmat(4,4) = 10;
    Dmat(5,5) = 20 + 0*std::abs(vel_angular_body.z);
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Dmat :\n" << Dmat);
    Eigen::VectorXd Dvec = -1.0*Dmat*state;
    ROS_DEBUG_STREAM_THROTTLE(1.0,"Dvec :\n" << Dvec);

    //Input
    Eigen::VectorXd inputVec = Eigen::VectorXd::Zero(6);
    inputVec(0) = SME.F(0);
    inputVec(1) = SME.F(1);
    inputVec(3) = SME.F(2);
    inputVec(5) = SME.F(3);
    ROS_DEBUG_STREAM_THROTTLE(1.0,"inputVec :\n" << inputVec);

    //ROS_INFO("input");

    // Restoring/Buoyancy Forces
    double buoy_force = (water_level_ - pose.pos.z)*0.48*g*water_density_;
    Eigen::VectorXd buoyVec = Eigen::VectorXd::Zero(6);
    buoyVec(2) = buoy_force;  // Z direction - shoudl really be in XYZ frame
    buoyVec(3) = -0.4*sin(euler.x)*buoy_force; // roll
    buoyVec(4) = -0.4*sin(euler.y)*buoy_force; // pitch
    ROS_DEBUG_STREAM_THROTTLE(1.0,"buoyVec :\n" << buoyVec);

    // Sum all forces
    // note, inputVec only includes torque component
    Eigen::VectorXd forceSum = inputVec + amassVec + buoyVec + Dvec;
    //ROS_INFO("sum");


    ROS_DEBUG_STREAM("forceSum :\n" << forceSum);
    math::Vector3 totalLinear(forceSum(0),forceSum(1),forceSum(2));
    math::Vector3 totalAngular(forceSum(3),forceSum(4),forceSum(5));

    // Add dynamic forces/torques to link at CG
    link_->AddRelativeForce(totalLinear);
    link_->AddRelativeTorque(totalAngular);

    //ROS_INFO("add force");

}

void SailboatPlugin::OnMachDrive( const sailboat_message::Mach_msg::ConstPtr &msg)
{
    last_mach_drive_time_ = this->world_->GetSimTime();
    last_mach_drive_sail_ = msg->sail;
    last_mach_drive_rudder_ = msg->rudder;

//    SME.rudderAngle = msg->rudder;
//    SME.sailAngle = msg->sail;
//    if(SME.sailAngle>1.5) SME.sailAngle=1.5;
//    if(SME.sailAngle<-1.5) SME.sailAngle=-1.5;
//    SME.delta_r = SME.rudderAngle;
//    SME.delta_s = SME.sailAngle;
}

void SailboatPlugin::OnWindDrive( const sailboat_message::Wind_Simulation_msg::ConstPtr &msg)
{
    last_wind_TWA_ = msg->TWA;
    last_wind_TWS_ = msg->TWS;
//    SME.windDirection = msg->TWA;
//    SME.windVelocity = msg->TWS;
}


void SailboatPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(SailboatPlugin);

