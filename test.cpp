#include <iostream>
#include "rl_build/include/mdl/Kinematic.h"
#include "rl_build/include/mdl/Model.h"
#include "rl_build/include/mdl/UrdfFactory.h"
#include "rl_build/include/mdl/NloptInverseKinematics.h"

int main() {
  rl::mdl::UrdfFactory factory;
  std::shared_ptr<rl::mdl::Model> model(
      factory.create("/home/derek/XR1_WS/src/gingerurdf/urdf/gingerurdf.urdf"));
  rl::mdl::Kinematic* kinematics =
      dynamic_cast<rl::mdl::Kinematic*>(model.get());
  auto dof = kinematics->getDof();
  auto pos = kinematics->getPosition();
  rl::math::Vector setVal(dof);
//   setVal << 0, 0, 1;
//   kinematics->setPosition(setVal);
  auto tau = kinematics->getTorque();
  rl::math::Transform t = kinematics->getOperationalPosition(0);
  std::cout << t.matrix() << std::endl;

  rl::mdl::NloptInverseKinematics ik(kinematics);
  ik.goals.push_back(::std::make_pair(t, 10));
  bool result = ik.solve();
  rl::math::Vector solution = kinematics->getPosition();
  std::cout << solution << std::endl;

}