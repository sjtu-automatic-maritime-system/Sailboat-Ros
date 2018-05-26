#ifndef SAILBOAT_GAZEBO_DYNAMICS_H
#define SAILBOAT_GAZEBO_DYNAMICS_H

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sdf/sdf.hh>

//ROS

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <Eigen/Core>
//#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/console.h>

//sailboat msgs
#include <sailboat_message/Mach_msg.h>
#include <sailboat_message/Wind_Simulation_msg.h>

#include <sailboat_simulation_lib/CSimulationVer1.h>

namespace gazebo
{



    class SailboatPlugin : public ModelPlugin
    {
    public:
        SailboatPlugin();
        virtual ~SailboatPlugin();
        /*! Loads the model in gets dynamic parameters from SDF. */
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    protected:
        /*! Callback for Gazebo simulation engine */
        virtual void UpdateChild();
        virtual void FiniChild();
    private:
        /*! Presumably this would get called when there is a collision,
          but not implemented! */
        void OnContact(const std::string &name, const physics::Contact &contact);
        /*!
          Callback for Sailboat Drive commands and Wind
          \param msg Sailboat Drive message and Wind simulation message
        */
        void OnMachDrive( const sailboat_message::Mach_msg::ConstPtr &msg);
        void OnWindDrive( const sailboat_message::Wind_Simulation_msg::ConstPtr &msg);

        /*! ROS spin once */
        void spin();

        /*! Convenience function for getting SDF parameters

         */
        double getSdfParamDouble(sdf::ElementPtr sdfPtr,const std::string &param_name,double default_val);

        /*! Takes ROS Sailboat Drive commands and scales them by max thrust

          \param cmd ROS drive command
          \param max_cmd  Maximum value expected for commands - scaling factor
          \param max_pos  Maximum positive force value
          \param max_neg  Maximum negative force value
          \return Value scaled and saturated
         */
        double scaleThrustCmd(double cmd, double max_cmd, double max_pos, double max_neg);

        /// Parameters

        /// sailboat simulation
        CSimulationVer1 SME;


        std::string node_namespace_;
        std::string link_name_;

        ros::NodeHandle *rosnode_;

//    ros::Publisher sensor_state_pub_;
//    ros::Publisher odom_pub_;
//    ros::Publisher joint_state_pub_;

        ros::Subscriber mach_drive_sub_;
        ros::Subscriber wind_drive_sub_;

        //GazeboRosPtr gazebo_ros_;

        event::ConnectionPtr update_connection_;

        /*! Pointer to the Gazebo world, retrieved when the model is loaded */
        physics::WorldPtr world_;
        /*! Pointer to Gazebo parent model, retrieved when the model is loaded */
        physics::ModelPtr model_;
        /*! Pointer to model link in gazebo,
          optionally specified by the bodyName parameter,
          The states are taken from this link and forces applied to this link.*/
        physics::LinkPtr link_;

        // Simulation time of the last update
        common::Time prev_update_time_;
        math::Vector3 prev_lin_vel_;
        math::Vector3 prev_ang_vel_;

        common::Time last_mach_drive_time_;
        double last_mach_drive_sail_;
        double last_mach_drive_rudder_;
        double last_mach_drive_motor_;
        double last_wind_TWA_;
        double last_wind_TWS_;
        double last_uu;
        double last_vv;
        double last_pp;
        double last_rr;
        double last_XX;
        double last_YY;
        double last_phi;
        double last_psi;

        /* Water height [m]*/
        double water_level_;
        /* Water density [kg/m^3] */
        double water_density_;
        /*! Timeout for recieving Drive commands [s]*/
        double cmd_timeout_;
        /*! Added mass matrix, 6x6 */
        Eigen::MatrixXd Ma_;

        double g;

        //sailboat
        double m; //kg; mass of sailboat
        //Sail
        double Asm;

        math::Pose pose;
        math::Vector3 euler;
        math::Vector3 velocity;
        math::Vector3 acceleration;
        math::Vector3 angular_velocity_;
        math::Vector3 angular_acceleration_;

        boost::thread *spinner_thread_;

        event::ConnectionPtr contact_event_;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

    };  // class SailboatPlugin
} // namespace gazebo

#endif //SAILBOAT_GAZEBO_DYNAMICS_H
