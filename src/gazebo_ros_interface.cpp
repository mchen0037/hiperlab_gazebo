//By Mighty Chen 6/29/18
//mchen73@ucmerced.edu

#include <gazebo_ros_interface.h>

namespace gazebo {
  //Line is necessary for all Gazebo Plugins
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosInterface);

  void GazeboRosInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model = _model;

    //initialize ROS
    if (!ros::isInitialized()) {
      int argc = 0;
      char ** argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    this->nh.reset(new ros::NodeHandle("gazebo_client"));

    //ROS Subscriber
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model->GetName() + "/cmd_vel",
      1,
      boost::bind(&GazeboRosInterface::TmpCallback, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->nh->subscribe(so);

    //publisher
    this->simulator_truth_pub = this->nh->advertise<hiperlab_rostools::simulator_truth>(
      "/" + this->model->GetName() + "simulator_truth",
      1);


    //handle ROS multi-threading
    this->rosQueueThread = std::thread(std::bind(&GazeboRosInterface::QueueThread, this));
    return;
  }

  void GazeboRosInterface::TmpCallback(const std_msgs::Float32ConstPtr &_msg) {
    std::cout << _msg->data << std::endl;
  }

  //Handle ROS multi-threading
  void GazeboRosInterface::QueueThread() {
    static const double timeout = 0.01;
    while(this->nh->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }
}
