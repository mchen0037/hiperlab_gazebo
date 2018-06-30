#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//USED TO HANDLE THE INDIVIDUAL ROTORS AND WHAT VELOCITIES TO APPLY TO THEM.

namespace gazebo {

class GazeboMotorPlugin : public ModelPlugin {
public:
    GazeboMotorPlugin() : ModelPlugin() {}

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);
  void velocity_gz_callback(ConstVector3dPtr &_msg);

private:
  std::string joint_name;
  event::ConnectionPtr updateConnection_;
  physics::ModelPtr model;
  physics::JointPtr joint;
  transport::NodePtr node;
  transport::SubscriberPtr sub;
  };
}
