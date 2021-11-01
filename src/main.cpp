#include <iostream>
#include "renderer.h"
#include "controller.h"
#include "simulation.h"
#include "ros_manager.h"


int main(int argc, char**argv) {
  ROSManager ros_manager;
  ros_manager.Init(argc, argv);
  constexpr std::size_t kFramesPerSecond{20};
  constexpr std::size_t kMsPerFrame{1000 / kFramesPerSecond};
  constexpr std::size_t kSimWidth{750};
  constexpr std::size_t kScreenHeight{750};
  constexpr std::size_t kInfoWidth{200};
  constexpr double maxLinearVel{10.0};
  constexpr double minLinearVel{-10.0};
  constexpr double maxAngularVel{1.0};
  constexpr double minAngularVel{-1.0};
  constexpr double linearStep{0.1};
  constexpr double angularStep{0.05};


  Renderer renderer(kSimWidth, kInfoWidth, kScreenHeight, ros_manager.GetPkgPath() );
  Controller controller(linearStep, angularStep);
  Simulation sim(minLinearVel, maxLinearVel, minAngularVel, maxAngularVel);

  sim.Run(controller, renderer, kMsPerFrame, ros_manager);

  std::cout << "Simulation has terminated successfully!\n";

  return 0;
}
