#include "gazebo_jebisim_rotor.h"
#include <boost/bind.hpp>

namespace gazebo {

GazeboJebiSimRotor::GazeboJebiSimRotor()
    : ModelPlugin()
{
}

GazeboJebiSimRotor::~GazeboJebiSimRotor() {
  updateConnection_->~Connection();
}

void GazeboJebiSimRotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

	model_ = _model;

	if(_sdf->HasElement("linkName"))
	  link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
	else
	  gzerr << "[gazebo_jebisim_rotor] Please specify a linkName.\n";

	link_ = model_->GetLink(link_name_);

	if(link_ == NULL)
   	  gzthrow("[gazebo_jebisim_rotor] Couldn't find specified link \"" << link_name_ << "\".");


	if(_sdf->HasElement("applyLinkName"))
	  applyLink_name_ = _sdf->GetElement("applyLinkName")->Get<std::string>();
	else
	  gzerr << "[gazebo_jebisim_rotor] Please specify a applyLinkName.\n";

	applyLink_ = model_->GetLink(applyLink_name_);



	this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJebiSimRotor::OnUpdate, this, _1));
}

void GazeboJebiSimRotor::OnUpdate(const common::UpdateInfo& _info) {

	#if GAZEBO_MAJOR_VERSION >= 9
	  T_W_I = link_->WorldPose();
	#else
	  T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
	#endif

	#if GAZEBO_MAJOR_VERSION >= 9
	  double roll = applyLink_->WorldPose().Rot().Roll();
	#else
	  double roll = applyLink_->GetWorldPose().Rot().Roll();
	#endif

	double pitch = 0.0;
	double yaw = T_W_I.Rot().Yaw();

	ignition::math::Pose3d apply;

	apply.Pos() = applyLink_->WorldPose().Pos();
	apply.Rot().Euler(roll, pitch,yaw);


	this->applyLink_->SetWorldPose(apply);

}


  GZ_REGISTER_MODEL_PLUGIN(GazeboJebiSimRotor)
}
