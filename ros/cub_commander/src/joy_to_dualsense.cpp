#include "cub_commander/joy_to_dualsense.hpp"

void JoyToDualSense::update(const sensor_msgs::msg::Joy &msg)
{
    joy = msg;
}

bool JoyToDualSense::up()
{
    return joy.axes[7] >= 1.0;
}
bool JoyToDualSense::down()
{
    return joy.axes[7] <= -1.0;
}
bool JoyToDualSense::left()
{
    return joy.axes[6] >= 1.0;
}
bool JoyToDualSense::right()
{
    return joy.axes[6] <= -1.0;
}

bool JoyToDualSense::cross()
{
    return joy.buttons[0];
}
bool JoyToDualSense::circle()
{
    return joy.buttons[1];
}
bool JoyToDualSense::triangle()
{
    return joy.buttons[2];
}
bool JoyToDualSense::square()
{
    return joy.buttons[3];
}

bool JoyToDualSense::create()
{
    return joy.buttons[8];
}
bool JoyToDualSense::option()
{
    return joy.buttons[9];
}

bool JoyToDualSense::l1()
{
    return joy.buttons[4];
}
bool JoyToDualSense::r1()
{
    return joy.buttons[5];
}
bool JoyToDualSense::l2()
{
    return joy.buttons[6];
}
bool JoyToDualSense::r2()
{
    return joy.buttons[7];
}

double JoyToDualSense::lx()
{
    return joy.axes[1];
}
double JoyToDualSense::ly()
{
    return joy.axes[0];
}
double JoyToDualSense::rx()
{
    return joy.axes[4];
}
double JoyToDualSense::ry()
{
    return joy.axes[3];
}
double JoyToDualSense::l2f()
{
    double value = joy.axes[2];
    if (value != 0.0)
        initial_L2f = false;

    return initial_L2f ? 0.0 : (1.0 - value) / 2.0;
}
double JoyToDualSense::r2f()
{
    double value = joy.axes[5];
    if (value != 0.0)
        initial_R2f = false;

    return initial_R2f ? 0.0 : (1.0 - value) / 2.0;
}