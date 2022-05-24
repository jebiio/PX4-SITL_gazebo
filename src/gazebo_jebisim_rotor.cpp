#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <stdio.h>

namespace gazebo
{
  class GazeboJebiSimRotor : public ModelPlugin
  {

    private:
    	std::string link_name_;
	std::string apply_link_name_;

	physics::LinkPtr link_;
	physics::LinkPtr applyLink_;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
	this->model = _parent;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboJebiSimRotor::OnUpdate, this));

	if (_sdf->HasElement("linkName"))
	link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
	else
	gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
	link_ = model->GetLink(link_name_);
	if (link_ == NULL)
	gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");


	if (_sdf->HasElement("applyLinkName"))
	apply_link_name_ = _sdf->GetElement("applyLinkName")->Get<std::string>();
	else
	gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
	applyLink_ = model->GetLink(apply_link_name_);
	if (applyLink_ == NULL)
	gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");



    }

    // Called by the world update start event
    public: void OnUpdate()
    {
	ignition::math::Pose3d pose_;

	pose_ = this->link_->WorldPose();


	this->applyLink_->SetWorldPose(pose_);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboJebiSimRotor)
}
