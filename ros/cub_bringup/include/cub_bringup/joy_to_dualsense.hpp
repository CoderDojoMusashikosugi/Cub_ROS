#ifndef CUB_BRINGUP__JOY_TO_DUALENSE_HPP_
#define CUB_BRINGUP__JOY_TO_DUALENSE_HPP_

#include "sensor_msgs/msg/joy.hpp"

class JoyToDualSense{
public:
  void update(const sensor_msgs::msg::Joy &msg);

  bool up();
  bool down();
  bool left();
  bool right();

  bool cross();
  bool circle();
  bool triangle();
  bool square();

  bool create();
  bool option();

  bool l1();
  bool r1();
  bool l2();
  bool r2();

  double lx();
  double ly();
  double rx();
  double ry();
  double l2f();
  double r2f();
private:
  bool initial_L2f=true;
  bool initial_R2f=true;
  sensor_msgs::msg::Joy joy;
};

#endif  // CUB_BRINGUP__JOY_TO_DUALENSE_HPP_