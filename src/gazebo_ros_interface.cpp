/*  The bridge to communicate beween several Gazebo plugins and ROS msgs.

    Created By Mighty Chen 6/29/18
    mchen73@ucmerced.edu
*/

#include <gazebo_ros_interface.h>

//FIXME: Handle simulation time and ROS publishing rate?

namespace gazebo {
  //Line is necessary for all Gazebo Plugins
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosInterface);

  void GazeboRosInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    this->number_of_rotors = _sdf->HasElement("numberOfRotors") ?
      _sdf->Get<int>("numberOfRotors") : 4;

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboRosInterface::OnUpdate, this, _1));

    //GAZEBO TRANSPORTATION SYSTEM
    gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
    this->gzNode->Init();

    //Gazebo Subscribers: Imu Plugin, etc.
    std::string imuTopicName = "~/" + this->model->GetName() + "cmd_vel";
    this->imu_gz_sub = this->gzNode->Subscribe(
      imuTopicName, &GazeboRosInterface::ImuCallback, this);


    //Gazebo Publishers stored in a map
    //Assuming that rotor_#_joint is the name for the rotors.
    for (int i = 0; i < this->number_of_rotors; ++i) {
      std::string jointName = "rotor_" + std::to_string(i) + "_joint";
      std::string topicName = "~/" + this->model->GetName() + "/" +
        jointName + "/motor_velocity";
      list_of_rotors.push_back(this->model->GetJoint(jointName));
      rotors_publishers.insert(std::pair<std::string, transport::PublisherPtr>
        (topicName, gzNode->Advertise<msgs::Vector3d>(topicName)));
    }

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
    this->telem_pub = this->nh->advertise<hiperlab_rostools::telemetry>(
      "/" + this->model->GetName() + "/telemetry", //FIXME: handle vehicle ID
      1);

    //handle ROS multi-threading
    this->rosQueueThread = std::thread(std::bind(&GazeboRosInterface::QueueThread, this));
    return;
  }

  void GazeboRosInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
    hiperlab_rostools::simulator_truth current_truth = GetCurrentTruth();
    hiperlab_rostools::telemetry current_telemetry = GetCurrentTelemetry();

    this->simulator_truth_pub.publish(current_truth);
    this->telem_pub.publish(current_telemetry);

    //Gazebo Publishers for the motor velocities, publish individualvalues to each rotor plugin
    for (std::map<std::string, transport::PublisherPtr>::iterator i =
       rotors_publishers.begin();
       i != rotors_publishers.end(); ++i) {
         //i->first is the name of the joint, i->second is the publisher
         transport::PublisherPtr rotor_pub = i->second;
         //FIXME: Make this send meaningful data lol
         gazebo::msgs::Vector3d msg;
         gazebo::msgs::Set(&msg, ignition::math::Vector3d(1, 0, 0));
         rotor_pub->Publish(msg);
    }
  }

  hiperlab_rostools::telemetry GetCurrentTelemetry() {
    //FIXME: Make a global value for current_telem that subscribes to
    //an IMU plugin.
  }

  void GazeboRosInterface::ImuCallback(ImuPtr &msg) {
    //FIXME: Write the plugin first
    std::cout << "Hello from ImuCallback!" << std::endl;
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
