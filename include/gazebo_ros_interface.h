#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

//for ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "hiperlab_rostools/simulator_truth.h"
#include "hiperlab_rostools/telemetry.h"
#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/radio_command.h"

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"
#include "Components/Logic/QuadcopterLogic.hpp"

//From Protobuf
#include "Imu.pb.h"
#include "CommandMotorSpeed.pb.h"

namespace gazebo {
//typedefs are convienent for protobuf messages.
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

class GazeboRosInterface : public ModelPlugin {
public:
    GazeboRosInterface() : ModelPlugin() {}

protected:
  //This function is called immediately when you run Gazebo.
  void Load(physics::ModelPtr _model, sdf::ElementPtr);
  //See updateConnection_. This function is called after every iteration of simulation.
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  //Please find a better way to handle vehicle ID. My QuadcopterConstants will take vehicleID 37 as the iris model.
  int vehicleID = 37;

  //Struct to handle vehicles within the simulator. Theoretically works if you
  //spawn more than one vehicle, but haven't tested it yet.
  struct SimVehicle {
    struct {
      std::shared_ptr<
          Simulation::CommunicationsDelay<
              RadioTypes::RadioMessageDecoded::RawMessage> > queue;
    } cmdRadioChannel;
    std::shared_ptr<Onboard::QuadcopterLogic> _logic;
  };

  std::shared_ptr<SimVehicle> vehicle;

  //Timers and Stuff mostly pulled from Lab Code.
  HardwareTimer simTimer;
  const double frequencySimulation = 500.0;
  const double frequencyROS = 200;
  double const timeDelayOffboardControlLoop = 20e-3;
  double timePublishROS = 0;

  float _battVoltage, _battCurrent;

  std::mutex cmdRadioChannelMutex;  //protect against concurrency problems
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType;

  std::shared_ptr<Timer> debugTimer;
  double timePrintNextInfo;

  std::shared_ptr<Timer> _timerOnboardLogic;
  double _onboardLogicPeriod;

  // Simulation::Quadcopter vehicle;
  int number_of_rotors;
  std::vector<gazebo::physics::JointPtr> list_of_rotors;

  //Gazebo Stuff.
  //ModelPtr keeps track of the quadcopter information (name, pose, etc.)
  physics::ModelPtr model;
  //Use this to connect to OnUpdate(), which is called every iteration of the simulation.
  event::ConnectionPtr updateConnection_;

  //Gazebo Publisher to publish motor speed. Motor Speed values are calculated
  // in vehicle->_logic.
  transport::NodePtr gzNode;
  transport::PublisherPtr cmd_motor_speed_pub;

  //Gazebo Subscribers
  transport::SubscriberPtr imu_gz_sub;
  void ImuCallback(ImuPtr& msg);
  Vec3f current_attitude, current_accelerometer, current_rateGyro;

  //ROS Stuff to handle multithreading
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::unique_ptr<ros::NodeHandle> nh;
  void QueueThread();

  //ROS Subscriber to recieve radio commands
  ros::Subscriber sub_radio_cmd;
  void RadioCmdCallback(const hiperlab_rostools::radio_command::ConstPtr &msg);

  //ROS Publisher for telemetry warnings
  ros::Publisher telem_pub;
  hiperlab_rostools::telemetry GetCurrentTelemetry();
  hiperlab_rostools::telemetry current_telemetry;

  //ROS Publisher for simulator truth
  ros::Publisher simulator_truth_pub;
  hiperlab_rostools::simulator_truth GetCurrentTruth();

  //ROS Publisher for mocap output (TODO: Mocap publishes same values as sim truth)
  ros::Publisher mocap_output_pub;
  hiperlab_rostools::mocap_output GetCurrentMocap();

  };
}
