#include "cub_commander/joy_to_xbox1914.hpp"

void JoyToXBox1914::update(const sensor_msgs::msg::Joy &msg)
{
    joy = msg;
}

bool JoyToXBox1914::up()
{
    return joy.axes[7] >= 1.0;
}
bool JoyToXBox1914::down()
{
    return joy.axes[7] <= -1.0;
}
bool JoyToXBox1914::left()
{
    return joy.axes[6] >= 1.0;
}
bool JoyToXBox1914::right()
{
    return joy.axes[6] <= -1.0;
}

bool JoyToXBox1914::y()
{
    return joy.buttons[4];
}
bool JoyToXBox1914::a()
{
    return joy.buttons[0];
}
bool JoyToXBox1914::x()
{
    return joy.buttons[3];
}
bool JoyToXBox1914::b()
{
    return joy.buttons[1];
}

bool JoyToXBox1914::view()
{
    return joy.buttons[10];
}
bool JoyToXBox1914::menu()
{
    return joy.buttons[11];
}
bool JoyToXBox1914::xbox()
{
    return joy.buttons[12];
}

bool JoyToXBox1914::l()
{
    return joy.buttons[6];
}
bool JoyToXBox1914::r()
{
    return joy.buttons[7];
}
bool JoyToXBox1914::lt()
{
    return joy.axes[5] <= -1.0;
}
bool JoyToXBox1914::rt()
{
    return joy.axes[4] <= -1.0;
}
bool JoyToXBox1914::l3()
{
    return joy.buttons[13];
}
bool JoyToXBox1914::r3()
{
    return joy.buttons[14];
}

double JoyToXBox1914::lx()
{
    return joy.axes[1];
}
double JoyToXBox1914::ly()
{
    return joy.axes[0];
}
double JoyToXBox1914::rx()
{
    return joy.axes[3];
}
double JoyToXBox1914::ry()
{
    return joy.axes[2];
}
double JoyToXBox1914::ltf()
{
    double value = joy.axes[5];
    if (value != 0.0)
        initial_L2f = false;

    return initial_L2f ? 0.0 : (1.0 - value) / 2.0;
}
double JoyToXBox1914::rtf()
{
    double value = joy.axes[4];
    if (value != 0.0)
        initial_R2f = false;

    return initial_R2f ? 0.0 : (1.0 - value) / 2.0;
}