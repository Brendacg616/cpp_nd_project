#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
using namespace std;
struct Velocity
{
  double linear{0};
  double angular{0};
};

struct Position{
  double x;
  double y;
  double z;
};

class Robot
{
  public:
    Robot(double min_linear, double max_linear, double min_angular, double max_angular);
    bool IncreaseLinearVel(double step);
    bool DecreaseLinearVel(double step);
    bool IncreaseAngularVel(double step);
    bool DecreaseAngularVel(double step);
    void GetVelocity (double &linear, double &angular) const;
    void GetVelocity (Velocity &vel) const;
    void GetPosition (double &x, double &y, double &z) const;
    void GetPosition (Position &pos) const;
    void SetPosition (Position &pos);
  private:
    struct Velocity max_vel, min_vel, current_vel;
    struct Position position;
};

#endif
