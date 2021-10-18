/** Author: Brenda Camacho **/
/** e-mail: brendacg616@gmail.com **/
/** Reference: "Linux Keyboard Events", http://www.cplusplus.com/forum/unices/190594/ **/
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "SDL.h"
#include "robot.h"

class Controller {
 public:
  Controller(double linear_step, double angular_step);
  void HandleInput(bool &running, Robot &robot) const;
  void IncreaseLinearVel (Robot &robot) const;
  void DecreaseLinearVel (Robot &robot) const;
  void IncreaseAngularVel (Robot &robot) const;
  void DecreaseAngularVel (Robot &robot) const;
private:
  double linear_step;
  double angular_step;

};

#endif
