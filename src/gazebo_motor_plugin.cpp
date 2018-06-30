/* Used to handle individual rotors and what velocities to spin them.
   Based on the velocities, Calculate the forces/thrust to apply to each rotor.
   This plugin needs to be applied to EACH rotor.

   Created by Mighty Chen 6/30/18
   mchen73@ucmerced.edu
*/

#include <gazebo_motor_plugin.h>

namespace gazebo{
  GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);

  void GazeboMotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    if(this->model->GetJointCount() == 0) {
      std::cout << "ERROR: No joints available." << std::endl;
      return;
    }

    if (_sdf->HasElement("jointName")) {
      this->joint_name = _sdf->Get<std::string>("jointName");
    } else {
      std::cout << "Please specify a joint name in the SDF file." << std::endl;
      return;
    }

    this->joint = this->model->GetJoint(this->joint_name);

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    std::string topicName = "~/" + this->model->GetName() + "/" +
      this->joint_name + "/motor_velocity";
    this->sub = this->node->Subscribe(topicName, &GazeboMotorPlugin::velocity_gz_callback, this);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboMotorPlugin::OnUpdate, this, _1));

  }

  void GazeboMotorPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {

  }

  //FIXME: Once you recieve a velocity msg, calculate force and apply the force.
  void GazeboMotorPlugin::velocity_gz_callback(ConstVector3dPtr &_msg) {
    std::cout << this->joint_name << " should spin at " << _msg->x() << std::endl;
  }

}
