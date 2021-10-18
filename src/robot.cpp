#include "robot.h"

Robot::Robot(double min_linear, double max_linear, double min_angular,
  double max_angular) {
    min_vel.linear = min_linear;
    max_vel.linear = max_linear;
    min_vel.angular = min_angular;
    max_vel.angular = max_angular;
  }

bool Robot::IncreaseLinearVel(double step)
{
  if  ((current_vel.linear +step) > max_vel.linear )
  {
    cerr << "Linear speed greater than the maximum permited" << endl;
    return false;
  } else
  {
      current_vel.linear += step;
      return true;
  }
}
bool Robot::DecreaseLinearVel(double step)
{
  if ((current_vel.linear - step) < min_vel.linear)
  {
    cerr << "Linear speed lower than the minimum permited" << endl;
    return false;
  } else
  {
    current_vel.linear -= step;
    return true;
  }
}

bool Robot::IncreaseAngularVel(double step)
{
if  ((current_vel.angular + step) > max_vel.angular )
  {
    cerr << "Angular speed greater than the maximum permited" << endl;
    return false;
  } else
  {
      current_vel.angular += step;
      return true;
  }
}

bool Robot::DecreaseAngularVel(double step)
{
  if ((current_vel.angular - step) < min_vel.angular)
  {
    cerr << "Angular speed lower than the minimum permited" << endl;
    return false;
  } else
  {
      current_vel.angular -= step;
      return true;
  }
}

void Robot::GetVelocity(double &linear, double &angular) const
{
  linear = current_vel.linear;
  angular = current_vel.angular;
  return;
}
void Robot::GetVelocity(Velocity &vel) const
{
  vel.linear = current_vel.linear;
  vel.angular = current_vel.angular;
  return;
}

void Robot::GetPosition(double &x, double &y, double &z) const
{
  x = position.x;
  y = position.y;
  z = position.z;
  return;
}
void Robot::GetPosition(Position &pos) const
{
  pos.x = position.x;
  pos.y = position.y;
  pos.z = position.z;
  return;
}

void Robot::SetPosition(Position &pos)
{
  position = pos;
}
