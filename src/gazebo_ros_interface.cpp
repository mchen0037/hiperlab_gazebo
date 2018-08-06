/*
The bridge to communicate beween several Gazebo plugins and ROS msgs.

    Created By Mighty Chen 6/29/18
    mchen73@ucmerced.edu
*/

#include <gazebo_ros_interface.h>

namespace gazebo {
  //Line is necessary for all Gazebo Plugins
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosInterface);

  void GazeboRosInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model = _model;

    //int vehicleID = 5; //FIXME: Handle Vehicle ID better, default is 37

    vehicle.reset(new GazeboRosInterface::SimVehicle());

    quadcopterType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleID);
    Onboard::QuadcopterConstants consts(quadcopterType);
    _battVoltage = consts.lowBatteryThreshold + 0.5;
    _battCurrent = -1.0;
    vehicle->_logic.reset(new Onboard::QuadcopterLogic(&simTimer, 1.0 / frequencySimulation));
    vehicle->_logic->Initialise(quadcopterType, vehicleID);

    //Initialize vectors for gyro
    current_attitude = Vec3f(0, 0, 0);
    current_accelerometer = Vec3f(0, 0, 0);
    current_rateGyro = Vec3f(0, 0, 0);

    //Initialize the queue to listen to radio commands
    vehicle->cmdRadioChannel.queue.reset(new Simulation::CommunicationsDelay<
                RadioTypes::RadioMessageDecoded::RawMessage>(
                &simTimer, timeDelayOffboardControlLoop));

    //Initialize Timers
    debugTimer.reset(new Timer(&simTimer));
    timePrintNextInfo = 0;

    _timerOnboardLogic.reset(new Timer(&simTimer));
    _onboardLogicPeriod = 1.0 / frequencySimulation;

    //Specify number of rotors in SDF file. If not present, default to 4.
    number_of_rotors = _sdf->HasElement("numberOfRotors") ?
      _sdf->Get<int>("numberOfRotors") : 4;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboRosInterface::OnUpdate, this, _1));

    /********************************************|
    |**      GAZEBO TRANSPORTATION SYSTEM      **|
    |********************************************/

    gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
    //using this->gzNode will break the code.
    gzNode->Init();

    // Gazebo Subscribers: Imu Plugin, etc.
    std::string imuTopicName = "~/" + model->GetName() + "/imu";
    std::string cmd_motor_speed_topic_name = "~/" + model->GetName() +
      "/gazebo/command/motor_speed";

    //Subscribe to IMU (gazebo_imu_plugin)
    imu_gz_sub = gzNode->Subscribe(
      imuTopicName, &GazeboRosInterface::ImuCallback, this);
    //Publish MotorSpeeds (from vehicle->_logic to gazebo_motor_model)
    cmd_motor_speed_pub = gzNode->Advertise<mav_msgs::msgs::CommandMotorSpeed>
      (cmd_motor_speed_topic_name, 1);

    /********************************************|
    |**               ROS SYSTEM               **|
    |********************************************/

    //init ROS
    if (!ros::isInitialized()) {
      int argc = 0;
      char ** argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    this->nh.reset(new ros::NodeHandle("gazebo_client"));

    //ROS Subscriber to Radio Command. Define the subscriber parameters and then sub.
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<hiperlab_rostools::radio_command>(
        "/radio_command" + std::to_string(vehicleID),
        1,
        boost::bind(&GazeboRosInterface::RadioCmdCallback, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->sub_radio_cmd = this->nh->subscribe(so);

    //ROS Publisher to simulator Truth
    this->simulator_truth_pub = this->nh->advertise<hiperlab_rostools::simulator_truth>(
      "/simulator_truth"  + std::to_string(vehicleID),
      1);
    //ROS Publisher to telemetry
    this->telem_pub = this->nh->advertise<hiperlab_rostools::telemetry>(
      "/telemetry" + std::to_string(vehicleID),
      1);
    //ROS Publisher to mocap_output (TODO: it is the same as simulator_truth, add noise)
    this->mocap_output_pub = this->nh->advertise<hiperlab_rostools::mocap_output>(
      "/mocap_output" + std::to_string(vehicleID),
      1);

    //handle ROS multi-threading
    this->rosQueueThread = std::thread(std::bind(&GazeboRosInterface::QueueThread, this));

    //to avoid getting the 0 vector, see #11 on git repo
    current_accelerometer[2] = 0.1;

    return;
  }

  void GazeboRosInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
    //To handle radio messages
    std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
    if (vehicle->cmdRadioChannel.queue->HaveNewMessage()) {
      RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
        vehicle->cmdRadioChannel.queue->GetMessage().raw);
      vehicle->_logic->SetRadioMessage(msg);
    }

    //Run the logic every 1/frequencySimulation seconds. Also publish motor speed values.
    if(_timerOnboardLogic->GetSeconds<double>() > _onboardLogicPeriod) {
      _timerOnboardLogic->AdjustTimeBySeconds(-_onboardLogicPeriod);

      //TODO: Set Battery Measurements X
      vehicle->_logic->SetBatteryMeasurement(_battVoltage, _battCurrent);

      vehicle->_logic->SetIMUMeasurementRateGyro(current_rateGyro[0],
                                        current_rateGyro[1],
                                        current_rateGyro[2]);

      vehicle->_logic->SetIMUMeasurementAccelerometer(current_accelerometer[0],
                                            current_accelerometer[1],
                                            current_accelerometer[2]);
      vehicle->_logic->Run();

      mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;
      for (int i = 0; i < number_of_rotors; ++i) {
        turning_velocities_msg.add_motor_speed(
          vehicle->_logic->GetMotorSpeedCmd(i)
        );
      }
      cmd_motor_speed_pub->Publish(turning_velocities_msg);
    }

    // for debugging
    // if (debugTimer->GetSeconds<double>() > timePrintNextInfo) {
    //   timePrintNextInfo += 1;
    //   vehicle->_logic->PrintStatus();
    // }

    //Publish to ROS every 1/frequencyROS seconds
    if (debugTimer->GetSeconds<double>() > timePublishROS) {
      timePublishROS += 1 / frequencyROS;

      hiperlab_rostools::simulator_truth current_truth = GetCurrentTruth();
      hiperlab_rostools::telemetry current_telemetry = GetCurrentTelemetry();
      hiperlab_rostools::mocap_output current_mocap = GetCurrentMocap();

      this->simulator_truth_pub.publish(current_truth);
      this->telem_pub.publish(current_telemetry);
      this->mocap_output_pub.publish(current_mocap);
    }
  }

  hiperlab_rostools::mocap_output GazeboRosInterface::GetCurrentMocap() {
    hiperlab_rostools::mocap_output msg;
    //TODO: noise? mocap system? not simulation truth?
    math::Pose current_pose = this->model->GetWorldPose();

    msg.vehicleID = vehicleID;

    msg.posx = current_pose.pos.x;
    msg.posy = current_pose.pos.y;
    msg.posz = current_pose.pos.z;

    msg.attq0 = current_pose.rot.w;
    msg.attq1 = current_pose.rot.x;
    msg.attq2 = current_pose.rot.y;
    msg.attq3 = current_pose.rot.z;

    math::Vector3 quatToEuler = current_pose.rot.GetAsEuler();
    msg.attroll = quatToEuler.x;
    msg.attpitch = quatToEuler.y;
    msg.attyaw = quatToEuler.z;

    msg.header.stamp = ros::Time::now();

    return msg;
  }

  //Handle IMU Messages from Gazebo Plugin
  void GazeboRosInterface::ImuCallback(ImuPtr &msg) {
    //Convert from Quaternion Msg -> Quaternion -> Eurler Angles
    msgs::Quaternion imu_msg_orientation = msg->orientation();
    math::Quaternion imu_orientation = gazebo::msgs::ConvertIgn(
      imu_msg_orientation);
    math::Vector3 quatToEuler = imu_orientation.GetAsEuler();
    current_attitude.x = quatToEuler.x;
    current_attitude.y = quatToEuler.y;
    current_attitude.z = quatToEuler.z;

    msgs::Vector3d imu_msg_lin_accel = msg->linear_acceleration();
    math::Vector3 imu_lin_accel = gazebo::msgs::ConvertIgn(
      imu_msg_lin_accel);
    current_accelerometer.x = imu_lin_accel.x;
    current_accelerometer.y = imu_lin_accel.y;
    current_accelerometer.z = imu_lin_accel.z;

    msgs::Vector3d imu_msg_gyro = msg->angular_velocity();
    math::Vector3 imu_gyro = gazebo::msgs::ConvertIgn(imu_msg_gyro);
    current_rateGyro.x = imu_gyro.x;
    current_rateGyro.y = imu_gyro.y;
    current_rateGyro.z = imu_gyro.z;
  }

  hiperlab_rostools::telemetry GazeboRosInterface::GetCurrentTelemetry() {
    hiperlab_rostools::telemetry telMsgOut;
    //Fill out the telemetry package
    TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
    vehicle->_logic->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2); //<><

    TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2;
    TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
    TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);

    telMsgOut.packetNumber = dataPacket1.packetNumber;
    for (int i = 0; i < 3; i++) {
      telMsgOut.accelerometer[i] = dataPacket1.accel[i];
      telMsgOut.rateGyro[i] = dataPacket1.gyro[i];
      telMsgOut.position[i] = dataPacket1.position[i];
    }

    for (int i = 0; i < 4; i++) {
      telMsgOut.motorForces[i] = dataPacket1.motorForces[i];
    }
    telMsgOut.batteryVoltage = dataPacket1.battVoltage;

    for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
        i++) {
      telMsgOut.debugVals[i] = dataPacket2.debugVals[i];
    }

    Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(
        Vec3f(dataPacket2.attitude[0], dataPacket2.attitude[1],
              dataPacket2.attitude[2])).ToEulerYPR();
    for (int i = 0; i < 3; i++) {
      telMsgOut.velocity[i] = dataPacket2.velocity[i];
      telMsgOut.attitude[i] = dataPacket2.attitude[i];
      telMsgOut.attitudeYPR[i] = attYPR[i];

    }
    telMsgOut.panicReason = dataPacket2.panicReason;
    telMsgOut.warnings = dataPacket2.warnings;

    telMsgOut.header.stamp = ros::Time::now();

    return telMsgOut;
  }

  //Convienent function to grab the Current Simulation Truth
  hiperlab_rostools::simulator_truth GazeboRosInterface::GetCurrentTruth() {
    math::Pose current_pose = this->model->GetWorldPose();

    hiperlab_rostools::simulator_truth current_truth;
    current_truth.posx = current_pose.pos.x;
    current_truth.posy = current_pose.pos.y;
    current_truth.posz = current_pose.pos.z;

    current_truth.attq0 = current_pose.rot.w;
    current_truth.attq1 = current_pose.rot.x;
    current_truth.attq2 = current_pose.rot.y;
    current_truth.attq3 = current_pose.rot.z;

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

    current_truth.vehicleID = vehicleID;

    current_truth.header.stamp = ros::Time::now();
    return current_truth;
  }

  //Callback function for whenever a radio command message is recieved.
  void GazeboRosInterface::RadioCmdCallback(const hiperlab_rostools::radio_command::ConstPtr &msg) {
    std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
    RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; ++i) {
      rawMsg.raw[i] = msg->raw[i];
    }
    vehicle->cmdRadioChannel.queue->AddMessage(rawMsg);
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
