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
    std::cout << ">>>>>>> gazebo_ros_interface successfully loaded <<<<" << std::endl;
    model = _model;

    quadcopterType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(5);
    _logic.reset(new Onboard::QuadcopterLogic(&simTimer, 1.0 / frequencySimulation));
    _logic->Initialise(quadcopterType, 5);

    debugTimer.reset(new Timer(&simTimer));
    timePrintNextInfo = 0;

    _timerOnboardLogic.reset(new Timer(&simTimer));
    _onboardLogicPeriod = 1.0 / frequencySimulation;

    number_of_rotors = _sdf->HasElement("numberOfRotors") ?
      _sdf->Get<int>("numberOfRotors") : 4;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboRosInterface::OnUpdate, this, _1));

    //GAZEBO TRANSPORTATION SYSTEM
    gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
    //for some reason using this->gzNode will break the code.
    gzNode->Init();

    // Gazebo Subscribers: Imu Plugin, etc.
    std::string imuTopicName = "~/" + model->GetName() + "/imu";
    std::string cmd_motor_speed_topic_name = "~/" + model->GetName() +
      "/gazebo/command/motor_speed";

    imu_gz_sub = gzNode->Subscribe(
      imuTopicName, &GazeboRosInterface::ImuCallback, this);
    gzNode->Advertise<mav_msgs::msgs::CommandMotorSpeed>
      (cmd_motor_speed_topic_name, 1);

    //initialize ROS
    if (!ros::isInitialized()) {
      int argc = 0;
      char ** argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    this->nh.reset(new ros::NodeHandle("gazebo_client"));

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<hiperlab_rostools::radio_command>(
        "/radio_command5",
        1,
        boost::bind(&GazeboRosInterface::RadioCmdCallback, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    this->sub_radio_cmd = this->nh->subscribe(so);

    //ROS PUBLISHERS
    this->simulator_truth_pub = this->nh->advertise<hiperlab_rostools::simulator_truth>(
      "/simulator_truth5",
      1);
    this->telem_pub = this->nh->advertise<hiperlab_rostools::telemetry>(
      "/telemetry5", //FIXME: handle vehicle ID
      1);
    this->mocap_output_pub = this->nh->advertise<hiperlab_rostools::mocap_output>(
      "/mocap_output5", //FIXME: handle vehicle ID
      1);

    //handle ROS multi-threading
    this->rosQueueThread = std::thread(std::bind(&GazeboRosInterface::QueueThread, this));
    return;
  }

  void GazeboRosInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
    if(_timerOnboardLogic->GetSeconds<double>() > _onboardLogicPeriod) {
      _timerOnboardLogic->AdjustTimeBySeconds(-_onboardLogicPeriod);

      //TODO: Set Battery Measurements X

      //TODO: SetIMUMeasuremenatRateGyro
      
      //TODO: SetIMUMeasurementAccelerometer

      _logic->Run();
    }

    //for debugging
    if (debugTimer->GetSeconds<double>() > timePrintNextInfo) {
      timePrintNextInfo += 1;
      _logic->PrintStatus();
    }

    hiperlab_rostools::simulator_truth current_truth = GetCurrentTruth();
    hiperlab_rostools::telemetry current_telemetry = GetCurrentTelemetry();
    hiperlab_rostools::mocap_output current_mocap = GetCurrentMocap();

    this->simulator_truth_pub.publish(current_truth);
    this->telem_pub.publish(current_telemetry);
    this->mocap_output_pub.publish(current_mocap);
  }

  hiperlab_rostools::mocap_output GazeboRosInterface::GetCurrentMocap() {
    hiperlab_rostools::mocap_output msg;
    //TODO: noise? mocap system? not simulation truth?
    math::Pose current_pose = this->model->GetWorldPose();

    msg.vehicleID = 5; //FIXME: handle vehicle id

    msg.posx = current_pose.pos.x;
    msg.posy = current_pose.pos.y;
    msg.posz = current_pose.pos.z;

    msg.attq0 = current_pose.rot.x;
    msg.attq1 = current_pose.rot.y;
    msg.attq2 = current_pose.rot.z;
    msg.attq3 = current_pose.rot.w;

    math::Vector3 quatToEuler = current_pose.rot.GetAsEuler();
    msg.attroll = quatToEuler.x;
    msg.attpitch = quatToEuler.y;
    msg.attyaw = quatToEuler.z;

    return msg;
  }

  //Handle IMU Messages from Gazebo Plugin
  void GazeboRosInterface::ImuCallback(ImuPtr &msg) {
    msgs::Quaternion imu_msg_orientation = msg->orientation();
    math::Quaternion imu_orientation = gazebo::msgs::ConvertIgn(
      imu_msg_orientation);
    math::Vector3 quatToEuler = imu_orientation.GetAsEuler();
    current_telemetry.attitude[0] = quatToEuler.x;
    current_telemetry.attitude[1] = quatToEuler.y;
    current_telemetry.attitude[2] = quatToEuler.z;

    msgs::Vector3d imu_msg_lin_accel = msg->linear_acceleration();
    math::Vector3 imu_lin_accel = gazebo::msgs::ConvertIgn(
      imu_msg_lin_accel);
    current_telemetry.accelerometer[0] = imu_lin_accel.x;
    current_telemetry.accelerometer[1] = imu_lin_accel.y;
    current_telemetry.accelerometer[2] = imu_lin_accel.z;

    msgs::Vector3d imu_msg_gyro = msg->angular_velocity();
    math::Vector3 imu_gyro = gazebo::msgs::ConvertIgn(imu_msg_gyro);
    current_telemetry.rateGyro[0] = imu_gyro.x;
    current_telemetry.rateGyro[1] = imu_gyro.y;
    current_telemetry.rateGyro[2] = imu_gyro.z;
  }

  hiperlab_rostools::telemetry GazeboRosInterface::GetCurrentTelemetry() {

    return current_telemetry;
  }

  //Convienent function to grab the Current Simulation Truth
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

  void GazeboRosInterface::RadioCmdCallback(const hiperlab_rostools::radio_command::ConstPtr &msg) {
    std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
    RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; ++i) {
      rawMsg.raw[i] = msg->raw[i];
    }
    cmdRadioChannel.queue->AddMessage(rawMsg);
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
