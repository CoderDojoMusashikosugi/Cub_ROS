#ifndef CUB_COMMANDER__JOYPAD_HPP_
#define CUB_COMMANDER__JOYPAD_HPP_

#include "sensor_msgs/msg/joy.hpp"
#include "cub_commander/joypad_base.hpp"
#include "cub_commander/joy_to_dualsense.hpp"
#include "cub_commander/joy_to_xbox1914.hpp"

class Joypad: public JoypadBase{
public:
  Joypad();
  void update(const sensor_msgs::msg::Joy &msg);

  void setDevice(std::string dev_name);
  void updateDevice();
  bool connected();

  bool up(){return device->up();};
  bool down(){return device->down();};
  bool left(){return device->left();};
  bool right(){return device->right();};

  bool buttonUp(){return device->buttonUp();};          // general
  bool triangle(){return buttonUp();};                  // PlayStation
  bool y(){return buttonUp();};                         // XBox

  bool buttonDown(){return device->buttonDown();};      // general
  bool cross(){return buttonDown();};                   // PlayStation
  bool a(){return buttonDown();};                       // XBox

  bool buttonLeft(){return device->buttonLeft();};      // general
  bool square(){return buttonLeft();};                  // PlayStation
  bool x(){return buttonLeft();};                       // XBox

  bool buttonRight(){return device->buttonRight();};    // general
  bool circle(){return buttonRight();};                 // PlayStation
  bool b(){return buttonRight();};                      // XBox

  bool optionLeft(){return device->optionLeft();};      // general
  bool create(){return optionLeft();};                  // PlayStation
  bool view(){return optionLeft();};                    // XBox

  bool optionRight(){return device->optionRight();};    // general
  bool option(){return optionRight();};                 // PlayStation
  bool menu(){return optionRight();};                   // XBox

  bool optionCenter(){return device->optionCenter();};  // general
  bool touchpad(){return optionCenter();};              // PlayStation
  bool share(){return optionCenter();};                 // XBox(not working)

  bool logo(){return device->logo();};                  // general
  bool ps(){return logo();};                            // PlayStation
  bool xbox(){return logo();};                          // XBox

  bool l1(){return device->l1();};                      // general, PlayStation
  bool l(){return l1();};                               // XBox

  bool r1(){return device->r1();};                      // general, PlayStation
  bool r(){return r1();};                               // XBox

  bool l2(){return device->l2();};                      // general, PlayStation
  bool lt(){return l2();};                              // XBox

  bool r2(){return device->r2();};                      // general, PlayStation
  bool rt(){return r2();};                              // XBox

  bool l3(){return device->l3();};                      // general, PlayStation, XBox
  bool r3(){return device->r3();};                      // general, PlayStation, XBox

  double lx(){return device->lx();};
  double ly(){return device->ly();};
  double rx(){return device->rx();};
  double ry(){return device->ry();};

  double l2f(){return device->l2f();};                  // general, PlayStation
  double ltf(){return l2f();};                          // XBox

  double r2f(){return device->r2f();};                  // general, PlayStation
  double rtf(){return r2f();};                          // XBox

  double last_recv_sec();
private:
  sensor_msgs::msg::Joy joy;
  JoyToDualSense dualsense;
  JoyToXBox1914 xbox1914;
  JoypadBase* device = &dualsense;
  std::string dev_name="/dev/input/js0";
  bool device_updated = false;
  std::shared_ptr<rclcpp::Clock> clock_;  
};

#endif  // CUB_COMMANDER__JOYPAD_HPP_