#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>

//for ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "hiperlab_rostools/simulator_truth.h"
#include "hiperlab_rostools/telemetry.h"
#include "hiperlab_rostools/mocap_output.h"

//From Protobuf
#include "Imu.pb.h"
#include "CommandMotorSpeed.pb.h"



namespace gazebo {
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

class GazeboRosInterface : public ModelPlugin {
public:
    GazeboRosInterface() : ModelPlugin() {}

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  int number_of_rotors;
  std::vector<gazebo::physics::JointPtr> list_of_rotors;

  //Gazebo Stuff
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection_;

  //Gazebo Transport Stuff
  transport::NodePtr gzNode;
  transport::PublisherPtr cmd_motor_speed_pub;

  //this was cool but kind of pointless now
  // std::map<std::string, transport::PublisherPtr> rotors_publishers;
  // std::vector<gazebo::transport::PublisherPtr> rotors_speed_pub;

  //Gazebo Subscribers
  transport::SubscriberPtr imu_gz_sub;
  void ImuCallback(ImuPtr& msg);

  //ROS Stuff
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::unique_ptr<ros::NodeHandle> nh;
  void QueueThread();

  ros::Subscriber rosSub;
  //Grab all 4 values from ROS, and send individual values to each rotor.
  void RotorsMotorSpeedCallback(const std_msgs::Float32ConstPtr &_msg);

  ros::Publisher telem_pub;
  hiperlab_rostools::telemetry GetCurrentTelemetry();
  hiperlab_rostools::telemetry current_telemetry;

  ros::Publisher simulator_truth_pub;
  hiperlab_rostools::simulator_truth GetCurrentTruth();

  ros::Publisher mocap_output_pub;
  hiperlab_rostools::mocap_output GetCurrentMocap();

  };
}
