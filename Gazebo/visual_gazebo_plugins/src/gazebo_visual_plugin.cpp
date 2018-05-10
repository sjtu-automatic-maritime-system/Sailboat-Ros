#include <visual_gazebo_plugins/gazebo_visual_plugin.h>



namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    SomeVisualPlugin::SomeVisualPlugin()
      //line(NULL),
    {
        num = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    SomeVisualPlugin::~SomeVisualPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void SomeVisualPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
        this->visual_ = _parent;

        this->visual_namespace_ = "visual/";

        // start ros node
        if (!ros::isInitialized())
        {
            int argc = 0;
            char** argv = NULL;
            ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        }

        //this->rosnode_ = new ros::NodeHandle(this->visited_visual_namespace_);
        this->rosnode_ = new ros::NodeHandle("visual");
        this->force_sub_ = this->rosnode_->subscribe("/some_force", 1000, &SomeVisualPlugin::VisualizeForceOnLink, this);


        ROS_INFO("start");

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectRender(
            boost::bind(&SomeVisualPlugin::UpdateChild, this));
        
        this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

        this->line->AddPoint(math::Vector3(-5, -5, 0.01));
        this->line->AddPoint(math::Vector3(-5, 5, 0.01));
        this->line->AddPoint(math::Vector3(-10, 5, 0.01));
        this->line->AddPoint(math::Vector3(-10, 20, 0.01));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void SomeVisualPlugin::UpdateChild()
    {

    //ROS_INFO("update");
    
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void SomeVisualPlugin::VisualizeForceOnLink(const geometry_msgs::PointConstPtr &force_msg)
    {
        //this->visual_->SetVisible(false);
        //this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
        // Add two points to a connecting line strip from (-5,-5,0.01) to (-5,5,0.01)
        if ( num%2 == 0){
        this->line->SetPoint(0,math::Vector3(-5, -5, 0.01));
        this->line->SetPoint(1,math::Vector3(-5, 5, 0.01));
        this->line->SetPoint(2,math::Vector3(-10, 5, 0.01));
        this->line->SetPoint(3,math::Vector3(-10, 20, 0.01));
        }
        else{
          this->line->SetPoint(0,math::Vector3(5, -5, 0.01));
          this->line->SetPoint(1,math::Vector3(5, 5, 0.01));
          this->line->SetPoint(2,math::Vector3(10, 5, 0.01));
          this->line->SetPoint(3,math::Vector3(10, 20, 0.01));

        }
        
        // set the Material of the line, in this case to purple
        this->line->setMaterial("Gazebo/Purple");
        this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
        this->visual_->SetVisible(true);
        ROS_INFO("draw");
        num += 1;
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(SomeVisualPlugin);
  }
}