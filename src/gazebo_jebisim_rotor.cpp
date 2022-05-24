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

	// std::cout << this->link_->WorldPose() << std::endl;

	this->applyLink_->SetWorldPose(this->link_->WorldPose());


}


  GZ_REGISTER_MODEL_PLUGIN(GazeboJebiSimRotor)
}
