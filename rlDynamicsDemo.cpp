#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include "rl_build/include/mdl/Dynamic.h"
#include "rl_build/include/mdl/XmlFactory.h"

#define TestNum 100000

int main(int argc, char** argv) {
  try {
    srand(std::random_device()());

    rl::mdl::XmlFactory factory;
    std::shared_ptr<rl::mdl::Model> model(
        factory.create("rl_build/xml_examples/rlmdl/unimation-puma560.xml"));

    rl::mdl::Dynamic* dynamic = dynamic_cast<rl::mdl::Dynamic*>(model.get());

    rl::math::Vector q0(model->getDofPosition());
    rl::math::Vector qd0(model->getDof());
    rl::math::Vector qdd0(model->getDof());

    for (::std::size_t i = 0; i < TestNum; ++i) {
      if (0 == i) {
        q0.setZero();
        qd0.setZero();
        qdd0.setOnes();
      } else {
        q0.setRandom();
        qd0.setRandom();
        qdd0.setRandom();
      }

      //   // forward velocity (recursive)

      //   dynamic->setPosition(q0);
      //   dynamic->setVelocity(qd0);
      //   dynamic->forwardVelocity();

      //   rl::math::Vector xdRecursive(6 * dynamic->getOperationalDof());

      //   for (::std::size_t j = 0; j < dynamic->getOperationalDof(); ++j) {
      //     xdRecursive.segment(6 * j, 3) =
      //         dynamic->getOperationalVelocity(j).linear();
      //     xdRecursive.segment(6 * j + 3, 3) =
      //         dynamic->getOperationalVelocity(j).angular();
      //   }

      //   // forward velocity (matrices)

      //   dynamic->setPosition(q0);
      //   dynamic->calculateJacobian();

      //   rl::math::Vector xdMatrices = dynamic->getJacobian() * qd0;

      //   if (!xdMatrices.isApprox(xdRecursive)) {
      //     std::cerr << "q0 = " << q0.transpose() << std::endl;
      //     std::cerr << "qd0 = " << qd0.transpose() << std::endl;
      //     std::cerr << "xd (recursive) = " << xdRecursive.transpose()
      //               << std::endl;
      //     std::cerr << "xd (matrices) = " << xdMatrices.transpose() <<
      //     std::endl; return EXIT_FAILURE;
      //   }

      //   // forward acceleration (recursive)

      //   dynamic->setPosition(q0);
      //   dynamic->setVelocity(qd0);
      //   dynamic->setAcceleration(qdd0);
      //   dynamic->forwardVelocity();
      //   dynamic->forwardAcceleration();

      //   rl::math::Vector xddRecursive(6 * dynamic->getOperationalDof());

      //   for (::std::size_t j = 0; j < dynamic->getOperationalDof(); ++j) {
      //     xddRecursive.segment(6 * j, 3) =
      //         dynamic->getOperationalAcceleration(j).linear();
      //     xddRecursive.segment(6 * j + 3, 3) =
      //         dynamic->getOperationalAcceleration(j).angular();
      //   }

      //   // forward acceleration (matrices)

      //   dynamic->setPosition(q0);
      //   dynamic->calculateJacobian();
      //   dynamic->setVelocity(qd0);
      //   dynamic->calculateJacobianDerivative();

      //   rl::math::Vector xddMatrices =
      //       dynamic->getJacobian() * qdd0 + dynamic->getJacobianDerivative();

      //   if (!xddMatrices.isApprox(xddRecursive)) {
      //     std::cerr << "q0 = " << q0.transpose() << std::endl;
      //     std::cerr << "qd0 = " << qd0.transpose() << std::endl;
      //     std::cerr << "qdd0 = " << qdd0.transpose() << std::endl;
      //     std::cerr << "xdd (recursive) = " << xddRecursive.transpose()
      //               << std::endl;
      //     std::cerr << "xdd (matrices) = " << xddMatrices.transpose()
      //               << std::endl;
      //     return EXIT_FAILURE;
      //   }

      // inverse dynamics (recursive)

      dynamic->setPosition(q0);
      dynamic->setVelocity(qd0);
      dynamic->setAcceleration(qdd0);
      dynamic->inverseDynamics();

      std::chrono::steady_clock::time_point startTime =
          std::chrono::steady_clock::now();

      rl::math::Vector tauRecursive = dynamic->getTorque();

      std::chrono::steady_clock::time_point stopTime =
          std::chrono::steady_clock::now();
      std::cout << "getTorque() "
                << std::chrono::duration_cast<std::chrono::duration<double>>(
                       stopTime - startTime)
                           .count() *
                       1000
                << " ms" << std::endl;
      //   // inverse dynamics (matrices)

      //   dynamic->setPosition(q0);
      //   dynamic->calculateMassMatrix();
      //   dynamic->setPosition(q0);
      //   dynamic->setVelocity(qd0);
      //   dynamic->calculateCentrifugalCoriolis();
      //   dynamic->setPosition(q0);
      //   dynamic->calculateGravity();

      //   rl::math::Vector tauMatrices = dynamic->getMassMatrix() * qdd0 +
      //                                  dynamic->getCentrifugalCoriolis() +
      //                                  dynamic->getGravity();

      //   if (!tauMatrices.isApprox(tauRecursive)) {
      //     std::cerr << "q0 = " << q0.transpose() << std::endl;
      //     std::cerr << "qd0 = " << qd0.transpose() << std::endl;
      //     std::cerr << "qdd0 = " << qdd0.transpose() << std::endl;
      //     std::cerr << "tau (recursive) = " << tauRecursive.transpose()
      //               << std::endl;
      //     std::cerr << "tau (matrices) = " << tauMatrices.transpose()
      //               << std::endl;
      //     return EXIT_FAILURE;
      //   }

      //   // forward dynamics (recursive)

      //   dynamic->setPosition(q0);
      //   dynamic->setVelocity(qd0);
      //   dynamic->setTorque(tauRecursive);
      //   dynamic->forwardDynamics();

      //   rl::math::Vector qdd1Recursive = dynamic->getAcceleration();

      //   if (!qdd1Recursive.isApprox(qdd0)) {
      //     std::cerr << "q0 = " << q0.transpose() << std::endl;
      //     std::cerr << "qd0 = " << qd0.transpose() << std::endl;
      //     std::cerr << "qdd0 = " << qdd0.transpose() << std::endl;
      //     std::cerr << "tau (recursive) = " << tauRecursive.transpose()
      //               << std::endl;
      //     std::cerr << "tau (matrices) = " << tauMatrices.transpose()
      //               << std::endl;
      //     std::cerr << "qdd1 (recursive) = " << qdd1Recursive.transpose()
      //               << std::endl;
      //     return EXIT_FAILURE;
      //   }

      //   // forward dynamics (matrices)

      //   dynamic->setPosition(q0);
      //   dynamic->calculateMassMatrixInverse();
      //   dynamic->setPosition(q0);
      //   dynamic->setVelocity(qd0);
      //   dynamic->calculateCentrifugalCoriolis();
      //   dynamic->setPosition(q0);
      //   dynamic->calculateGravity();

      //   rl::math::Vector qdd1Matrices =
      //       dynamic->getMassMatrixInverse() *
      //       (tauMatrices - dynamic->getCentrifugalCoriolis() -
      //        dynamic->getGravity());

      //   if (!qdd1Matrices.isApprox(qdd1Recursive)) {
      //     std::cerr << "q0 = " << q0.transpose() << std::endl;
      //     std::cerr << "qd0 = " << qd0.transpose() << std::endl;
      //     std::cerr << "qdd0 = " << qdd0.transpose() << std::endl;
      //     std::cerr << "tau (recursive) = " << tauRecursive.transpose()
      //               << std::endl;
      //     std::cerr << "tau (matrices) = " << tauMatrices.transpose()
      //               << std::endl;
      //     std::cerr << "qdd1 (recursive) = " << qdd1Recursive.transpose()
      //               << std::endl;
      //     std::cerr << "qdd1 (matrices) = " << qdd1Matrices.transpose()
      //               << std::endl;
      //     return EXIT_FAILURE;
      //   }
    }
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}