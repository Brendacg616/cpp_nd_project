#include "controller.h"
#include <iostream>
#include "SDL.h"
#include "robot.h"

Controller::Controller(double linear_step, double angular_step):
linear_step(linear_step), angular_step(angular_step){
  std::cout << "Controller constructor" << std::endl;
}

void Controller::HandleInput(bool &running, Robot &robot) const{
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    if (e.type == SDL_QUIT) {
      running = false;
    } else if (e.type == SDL_KEYDOWN) {
      switch (e.key.keysym.sym) {
        case SDLK_UP:
            //TODO: Change to bool and send a waning if max vel
            IncreaseLinearVel(robot);
          break;

        case SDLK_DOWN:
            DecreaseLinearVel(robot);
          break;

        case SDLK_LEFT:
            IncreaseAngularVel(robot);
          break;

        case SDLK_RIGHT:
            DecreaseAngularVel(robot);
          break;
        case SDLK_0:
            StopRobot(robot);
          break;
      }
    }
  }
}

void Controller::IncreaseLinearVel(Robot &robot) const
{
  robot.IncreaseLinearVel(linear_step);
}

void Controller::DecreaseLinearVel(Robot &robot) const
{
  robot.DecreaseLinearVel(linear_step);
}

void Controller::IncreaseAngularVel(Robot &robot) const
{
  robot.IncreaseAngularVel(angular_step);
}

void Controller::DecreaseAngularVel(Robot &robot) const
{
  robot.DecreaseAngularVel(angular_step);
}

void Controller::StopRobot(Robot &robot) const
{
  robot.Stop();
}