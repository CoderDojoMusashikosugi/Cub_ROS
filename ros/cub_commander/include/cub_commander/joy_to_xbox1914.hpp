#ifndef CUB_COMMANDER__JOY_TO_XBOX1914_HPP_
#define CUB_COMMANDER__JOY_TO_XBOX1914_HPP_

#include "sensor_msgs/msg/joy.hpp"
#include "cub_commander/joypad_base.hpp"

class JoyToXBox1914:public JoypadBase{
public:
  void update(const sensor_msgs::msg::Joy &msg);

  bool up();
  bool down();
  bool left();
  bool right();

  bool y();
  bool a();
  bool x();
  bool b();

  bool view();
  bool menu();
  bool share(){return false;};
  bool xbox();

  bool l();
  bool r();
  bool lt();
  bool rt();
  bool l3();
  bool r3();

  double lx();
  double ly();
  double rx();
  double ry();
  double ltf();
  double rtf();

  // bool up(){return false;};
  // bool down(){return false;};
  // bool left(){return false;};
  // bool right(){return false;};

  bool buttonUp(){return y();};
  bool buttonDown(){return a();};
  bool buttonLeft(){return x();};
  bool buttonRight(){return b();};
  bool optionLeft(){return view();};
  bool optionRight(){return menu();};
  bool optionCenter(){return false;};
  bool logo(){return xbox();};

  bool l1(){return l();};
  bool r1(){return r();};
  bool l2(){return lt();};
  bool r2(){return rt();};
  // bool l3(){return false;};
  // bool r3(){return false;};

  // double lx(){return 0;};
  // double ly(){return 0;};
  // double rx(){return 0;};
  // double ry(){return 0;};

  double l2f(){return ltf();};
  double r2f(){return rtf();};

private:
  bool initial_L2f=true;
  bool initial_R2f=true;
  sensor_msgs::msg::Joy joy;
};

#endif  // CUB_COMMANDER__JOY_TO_XBOX1914_HPP_