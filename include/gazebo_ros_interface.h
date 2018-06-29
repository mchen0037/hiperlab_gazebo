#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>

//for ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "hiperlab_rostools/simulator_truth.h"

namespace gazebo {
 
class GazeboRosInterface : public ModelPlugin {
public:
    GazeboRosInterface() : ModelPlugin() {}

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr);
  void OnUpdate();

private:
  void QueueThread();
  void TmpCallback(const std_msgs::Float32ConstPtr &_msg);
  private: physics::ModelPtr model;
  ros::Subscriber rosSub;
  ros::Publisher simulator_truth_pub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::unique_ptr<ros::NodeHandle> nh;
  };
}
