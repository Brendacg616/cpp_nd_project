#include "simulation.h"
#include <iostream>
#include "SDL.h"

Simulation::Simulation (double min_linear, double max_linear,
  double min_angular, double max_angular): robot(min_linear, max_linear,
    min_angular, max_angular){
      std::cout << "Simulation constructor" << std::endl;
      route = new std::deque<Position>;

    }

void Simulation::Run(Controller const &controller, Renderer &renderer,
               std::size_t target_frame_duration, ROSManager & ros_manager) {
  std::cout << "Simulation::Run function" << std::endl;
  Uint32 title_timestamp = SDL_GetTicks();
  Uint32 frame_start;
  Uint32 frame_end;
  Uint32 frame_duration;
  int frame_count = 0;
  bool running = true;

  while (running) {
    frame_start = SDL_GetTicks();

    // Input, Update, Render - the main game loop.
    controller.HandleInput(running, robot);
    Update(ros_manager);
    renderer.Render(robot, route);

    frame_end = SDL_GetTicks();

    // Keep track of how long each loop through the input/update/render cycle
    // takes.
    frame_count++;
    frame_duration = frame_end - frame_start;

    // After every second, update the window title.
    if (frame_end - title_timestamp >= 1000) {
      renderer.UpdateWindowTitle(frame_count);
      frame_count = 0;
      title_timestamp = frame_end;

    }

    // If the time for this frame is too small (i.e. frame_duration is
    // smaller than the target ms_per_frame), delay the loop to
    // achieve the correct frame rate.
    if (frame_duration < target_frame_duration) {
      SDL_Delay(target_frame_duration - frame_duration);
    }
  }
}

void Simulation::Update(ROSManager & ros_manager) {
  Velocity vel;
  Position pos;
  robot.GetVelocity(vel);
  ros_manager.PublishVelocity(vel.linear, vel.angular);
  ros_manager.GetPosition(pos.x, pos.y, pos.z);

  robot.SetPosition(pos);
  UpdateDeque();
  ros_manager.Spin();
}

void Simulation::UpdateDeque()
{
  Position pos, last_pos;
  robot.GetPosition(pos);
  if (route->size()> 0)
  {
    last_pos =  route->back();
    if (last_pos.x != pos.x || last_pos.y != pos.y || last_pos.z != pos.z)
    {
      route->push_back(pos);
      if (route->size() > 1000)
        route->pop_front();
    }
  }
  else 
  {
    route->push_back(pos);
  }
}

Simulation::~Simulation ()
{
  delete route;
}
