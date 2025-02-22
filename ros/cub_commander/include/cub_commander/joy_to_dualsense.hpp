#ifndef CUB_COMMANDER__JOY_TO_DUALENSE_HPP_
#define CUB_COMMANDER__JOY_TO_DUALENSE_HPP_

#include "sensor_msgs/msg/joy.hpp"
#include "cub_commander/joypad_base.hpp"

class JoyToDualSense:public JoypadBase{
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
  bool touchpad();
  bool ps();

  bool l1();
  bool r1();
  bool l2();
  bool r2();
  bool l3();
  bool r3();

  double lx();
  double ly();
  double rx();
  double ry();
  double l2f();
  double r2f();

  // bool up(){return false;};
  // bool down(){return false;};
  // bool left(){return false;};
  // bool right(){return false;};

  bool buttonUp(){return triangle();};
  bool buttonDown(){return cross();};
  bool buttonLeft(){return square();};
  bool buttonRight(){return circle();};
  bool optionLeft(){return create();};
  bool optionRight(){return option();};
  bool optionCenter(){return touchpad();};
  bool logo(){return ps();};

  // bool l1(){return false;};
  // bool r1(){return false;};
  // bool l2(){return false;};
  // bool r2(){return false;};
  // bool l3(){return false;};
  // bool r3(){return false;};

  // double lx(){return 0;};
  // double ly(){return 0;};
  // double rx(){return 0;};
  // double ry(){return 0;};

  // double l2f(){return 0.0;};
  // double r2f(){return 0.0;};
private:
  bool initial_L2f=true;
  bool initial_R2f=true;
  sensor_msgs::msg::Joy joy;
};

#endif  // CUB_COMMANDER__JOY_TO_DUALENSE_HPP_