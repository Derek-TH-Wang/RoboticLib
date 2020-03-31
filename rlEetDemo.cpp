#include <boost/lexical_cast.hpp>
#include <iostream>
#include <memory>
#include <stdexcept>
#include "rl_build/include/kin/Kinematics.h"
#include "rl_build/include/math/Unit.h"
#include "rl_build/include/plan/DistanceModel.h"
#include "rl_build/include/plan/Eet.h"
#include "rl_build/include/plan/LinearNearestNeighbors.h"
#include "rl_build/include/plan/UniformSampler.h"
#include "rl_build/include/plan/WorkspaceSphereExplorer.h"
#include "rl_build/include/sg/Model.h"
#include "rl_build/include/sg/solid/Scene.h"

int main(int argc, char** argv) {
  try {
    std::shared_ptr<rl::sg::Scene> scene;
    scene = std::make_shared<rl::sg::solid::Scene>();

    scene->load(
        "rl_build/xml_examples/rlsg/unimation-puma560_boxes.convex.xml");

    std::shared_ptr<rl::kin::Kinematics> kinematics(rl::kin::Kinematics::create(
        "rl_build/xml_examples/rlkin/unimation-puma560.xml"));

    rl::math::Transform world = rl::math::Transform::Identity();
    world = rl::math::AngleAxis(
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

    Eigen::Matrix<rl::math::Unit, Eigen::Dynamic, 1> qUnits(
        kinematics->getDof());
    kinematics->getPositionUnits(qUnits);

    rl::math::Vector start(kinematics->getDof());
    start << 110, -200, 60, 0, 0, 0;
    start *= rl::math::DEG2RAD;

    rl::math::Vector goal(kinematics->getDof());
    goal << -20, 0, 90, -40, 0, 0;
    goal *= rl::math::DEG2RAD;

    rl::plan::DistanceModel model;
    model.kin = kinematics.get();
    model.model = scene->getModel(0);
    model.scene = scene.get();

    rl::plan::WorkspaceSphereExplorer explorer;
    rl::plan::Eet::ExplorerSetup explorerSetup;
    rl::plan::LinearNearestNeighbors nearestNeighbors(&model);
    rl::plan::Eet planner;
    rl::plan::UniformSampler sampler;

    rl::math::Vector3 explorerGoal;
    rl::math::Vector3 explorerStart;

    explorer.seed(0);
    planner.seed(0);
    sampler.seed(0);

    explorer.goal = &explorerGoal;
    explorer.greedy = rl::plan::WorkspaceSphereExplorer::GREEDY_SPACE;
    explorer.model = &model;
    explorer.radius = 0.025;
    explorer.range = 45;
    explorer.samples = 100;
    explorer.start = &explorerStart;

    explorerSetup.goalConfiguration = &goal;
    explorerSetup.goalFrame = -1;
    explorerSetup.startConfiguration = &start;
    explorerSetup.startFrame = -1;

    planner.alpha = 0.01f;
    planner.alternativeDistanceComputation = false;
    planner.beta = 0;
    planner.delta = 1.0f * rl::math::DEG2RAD;
    planner.distanceWeight = 0.1f;
    planner.epsilon = 1.0e-9f;
    planner.explorers.push_back(&explorer);
    planner.explorersSetup.push_back(explorerSetup);
    planner.gamma = 1.0f / 3.0f;
    planner.goal = &goal;
    planner.goalEpsilon = 0.1f;
    planner.goalEpsilonUseOrientation = false;
    planner.max.x() = 30;
    planner.max.y() = 30;
    planner.max.z() = 2;
    planner.model = &model;
    planner.min.x() = 0;
    planner.min.y() = 0;
    planner.min.z() = 0;
    planner.setNearestNeighbors(&nearestNeighbors, 0);
    planner.sampler = &sampler;
    planner.start = &start;
    planner.duration = (std::chrono::_V2::steady_clock::duration)1000000000 * 5;

    sampler.model = &model;

    std::cout << "solve() ... " << std::endl;
    std::chrono::steady_clock::time_point startTime =
        std::chrono::steady_clock::now();
    bool solved = planner.solve();
    std::chrono::steady_clock::time_point stopTime =
        std::chrono::steady_clock::now();
    std::cout << "solve() " << (solved ? "true" : "false") << " "
              << std::chrono::duration_cast<std::chrono::duration<double>>(
                     stopTime - startTime)
                         .count() *
                     1000
              << " ms" << std::endl;
    std::cout << "NumVertices: " << planner.getNumVertices()
              << "  NumEdges: " << planner.getNumEdges() << std::endl;

    if (solved) {
      if (boost::lexical_cast<std::size_t>(281) >= planner.getNumVertices() &&
          boost::lexical_cast<std::size_t>(280) >= planner.getNumEdges()) {
        return EXIT_SUCCESS;
      } else {
        std::cerr << "NumVertices and NumEdges are more than expected for this "
                     "test case.";
        return EXIT_FAILURE;
      }
    }

    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
