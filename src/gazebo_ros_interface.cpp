//By Mighty Chen 6/29/18
//mchen73@ucmerced.edu

#include <gazebo_ros_interface.h>

//FIXME: Handle simulation time and ROS publishing rate?

namespace gazebo {
  //Line is necessary for all Gazebo Plugins
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosInterface);

  void GazeboRosInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboRosInterface::OnUpdate, this, _1));

    //GAZEBO PUBLISHERS

    //initialize ROS
    if (!ros::isInitialized()) {
      int argc = 0;
      char ** argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    this->nh.reset(new ros::NodeHandle("gazebo_client"));

    //ROS SUBSCRIBERS FIXME: std_msgs::Float32 will change to an array for actuator values
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model->GetName() + "/rotors_motor_speed",
      1,
      boost::bind(&GazeboRosInterface::TmpCallback, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->nh->subscribe(so);

    //ROS PUBLISHERS
    this->simulator_truth_pub = this->nh->advertise<hiperlab_rostools::simulator_truth>(
      "/" + this->model->GetName() + "/simulator_truth",
      1);

    //handle ROS multi-threading
    this->rosQueueThread = std::thread(std::bind(&GazeboRosInterface::QueueThread, this));
    return;
  }

  void GazeboRosInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
    hiperlab_rostools::simulator_truth current_truth = GetCurrentTruth();
    this->simulator_truth_pub.publish(current_truth);
  }

  hiperlab_rostools::simulator_truth GazeboRosInterface::GetCurrentTruth() {
    math::Pose current_pose = this->model->GetWorldPose();

    hiperlab_rostools::simulator_truth current_truth;
    current_truth.posx = current_pose.pos.x;
    current_truth.posy = current_pose.pos.y;
    current_truth.posz = current_pose.pos.z;

    //FIXME: check if q0 is x or w in Quaternion. see http://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1math_1_1Quaternion.html
    current_truth.attq0 = current_pose.rot.x;
    current_truth.attq1 = current_pose.rot.y;
    current_truth.attq2 = current_pose.rot.z;
    current_truth.attq3 = current_pose.rot.w;

    //FIXME: double check this one also x, y, z = roll, pitch, yaw?
    math::Vector3 quatToEuler = current_pose.rot.GetAsEuler();
    current_truth.attroll = quatToEuler.x;
    current_truth.attpitch = quatToEuler.y;
    current_truth.attyaw = quatToEuler.z;

    math::Vector3 current_lin_vel = this->model->GetWorldLinearVel();
    current_truth.velx = current_lin_vel.x;
    current_truth.vely = current_lin_vel.y;
    current_truth.velz = current_lin_vel.z;

    math::Vector3 current_ang_vel = this->model->GetWorldAngularVel();
    current_truth.angvelx = current_ang_vel.x;
    current_truth.angvely = current_ang_vel.y;
    current_truth.angvelz = current_ang_vel.z;

    //FIXME: Handle vehicleID
    current_truth.vehicleID = 5;
    return current_truth;
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
