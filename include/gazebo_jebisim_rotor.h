#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <stdio.h>
#include <ignition/math/Quaternion.hh>

namespace gazebo {

class GazeboJebiSimRotor : public ModelPlugin {
 public:

 GazeboJebiSimRotor();
 ~GazeboJebiSimRotor();

 protected:
 	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo&);

 private:
    	std::string link_name_;
	std::string applyLink_name_;

	physics::LinkPtr link_;
	physics::LinkPtr applyLink_;

	physics::ModelPtr model_;

	ignition::math::Pose3d T_W_I;

	event::ConnectionPtr updateConnection_;

};

}
