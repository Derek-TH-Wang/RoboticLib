#include <boost/lexical_cast.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>
#include "rl_build/include/kin/Kinematics.h"
#include "rl_build/include/math/Unit.h"
#include "rl_build/include/plan/KdtreeNearestNeighbors.h"
#include "rl_build/include/plan/RrtConCon.h"
#include "rl_build/include/plan/RecursiveVerifier.h"
#include "rl_build/include/plan/SimpleModel.h"
#include "rl_build/include/plan/SimpleOptimizer.h"
#include "rl_build/include/plan/UniformSampler.h"
#include "rl_build/include/sg/ode/Model.h"
#include "rl_build/include/sg/ode/Scene.h"

int main(int argc, char** argv) {

  try {
    rl::sg::ode::Scene scene;
    scene.load("rl_build/xml_examples/rlsg/unimation-puma560_boxes.convex.xml");

    std::shared_ptr<rl::kin::Kinematics> kinematics(rl::kin::Kinematics::create(
        "rl_build/xml_examples/rlkin/unimation-puma560.xml"));

    rl::math::Transform world = rl::math::Transform::Identity();
    world =
        rl::math::AngleAxis(
            boost::lexical_cast<rl::math::Real>(90) * ::rl::math::DEG2RAD,
            ::rl::math::Vector3::UnitZ()) *
        ::rl::math::AngleAxis(
            boost::lexical_cast<rl::math::Real>(0) * ::rl::math::DEG2RAD,
            ::rl::math::Vector3::UnitY()) *
        ::rl::math::AngleAxis(
            boost::lexical_cast<rl::math::Real>(0) * ::rl::math::DEG2RAD,
            ::rl::math::Vector3::UnitX());
    world.translation().x() = boost::lexical_cast<rl::math::Real>(0);
    world.translation().y() = boost::lexical_cast<rl::math::Real>(0);
    world.translation().z() = boost::lexical_cast<rl::math::Real>(0);
    
    kinematics->world() = world;

    rl::plan::SimpleModel model;
    model.kin = kinematics.get();
    model.model = scene.getModel(0);
    model.scene = &scene;

    rl::plan::KdtreeNearestNeighbors nearestNeighbors0(&model);
    rl::plan::KdtreeNearestNeighbors nearestNeighbors1(&model);
    rl::plan::RrtConCon planner;
    rl::plan::UniformSampler sampler;

    planner.model = &model;
    planner.setNearestNeighbors(&nearestNeighbors0, 0);
	planner.setNearestNeighbors(&nearestNeighbors1, 1);
    planner.sampler = &sampler;
    planner.delta = 1.0*rl::math::DEG2RAD;
    planner.duration = (std::chrono::_V2::steady_clock::duration)1000000000 * 5;
    planner.epsilon = 0.01;

    sampler.model = &model;


    rl::math::Vector start(kinematics->getDof());
    start << 110, -200, 60, 0, 0, 0;
    start *= rl::math::DEG2RAD;
    planner.start = &start;
    std::cout << "start = " << *(planner.start) << std::endl;

    rl::math::Vector goal(kinematics->getDof());
    goal << -20, 0, 90, -40, 0, 0;
    goal *= rl::math::DEG2RAD;
    planner.goal = &goal;
    std::cout << "goal = " << *(planner.goal) << std::endl;

    std::cout << "construct() ... " << std::endl;
    std::chrono::steady_clock::time_point startTime =
        std::chrono::steady_clock::now();
    // planner.construct(15);
    std::chrono::steady_clock::time_point stopTime =
        std::chrono::steady_clock::now();
    std::cout << "construct() "
              << std::chrono::duration_cast<std::chrono::duration<double>>(
                     stopTime - startTime)
                         .count() *
                     1000
              << " ms" << std::endl;

    std::cout << "solve() ... " << std::endl;
    startTime = std::chrono::steady_clock::now();
    bool solved = planner.solve();
    stopTime = std::chrono::steady_clock::now();
    std::cout << "solve() " << (solved ? "true" : "false") << " "
              << std::chrono::duration_cast<std::chrono::duration<double>>(
                     stopTime - startTime)
                         .count() *
                     1000
              << " ms" << std::endl;
    auto path = planner.getPath();
    std::cout << "path size = " << path.size() << std::endl;
    for (auto &i : path) {
        std::cout << " " << i;
    }
    
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
