#ifndef SIMULATION_H
#define SIMULATION_H

#include "SDL.h"
#include "controller.h"
#include "renderer.h"
#include "robot.h"
#include "ros_manager.h"
#include <deque>

class Simulation {
 public:
   Simulation (double min_linear, double max_linear, double min_angular, double max_angular);
  void Run(Controller const &controller, Renderer &renderer,
           std::size_t target_frame_duration, ROSManager & ros_manager);
  ~Simulation();
 private:
  Robot robot;
  std::deque<Position> *route;
  void Update(ROSManager & ros_manager);
  void UpdateDeque();
};

#endif
