#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <linux/joystick.h>

#include <rclcpp/rclcpp.hpp>
#include "cub_commander/joypad.hpp"

#include "cub_commander/joy_to_xbox1914.hpp"
#include "cub_commander/joy_to_dualsense.hpp"

Joypad::Joypad(){
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

void Joypad::update(const sensor_msgs::msg::Joy &msg)
{
    if(!device_type_overridden && !connected()) updateDevice();
    joy = msg;
    dualsense.update(joy);
    xbox1914.update(joy);
    
}

void Joypad::setDevice(std::string dev_name){
    this->dev_name = dev_name;
}

bool Joypad::setDeviceType(const std::string &device_type){
    if(device_type == "xbox"){
        device = &xbox1914;
        device_type_overridden = true;
        return true;
    }
    if(device_type == "dualsense"){
        device = &dualsense;
        device_type_overridden = true;
        return true;
    }
    if(device_type == "auto"){
        device_type_overridden = false;
        device_updated = false;
        return true;
    }
    return false;
}

void Joypad::updateDevice(){
    int fd = open (dev_name.c_str(), O_RDONLY);
	if(fd < 0){
		return;
	}
	char name[128];
	if (ioctl(fd, JSIOCGNAME(sizeof(name)), name) < 0)
		strncpy(name, "Unknown", sizeof(name));
	close(fd);
    std::string typeString = std::string(name);
    printf("detected device: %s\n",name);    
    if(typeString=="Xbox Wireless Controller"){
        printf("JoyToXBox1914\n");
        device = &xbox1914;
    }else{//PlayStation & others
        printf("JoyToDualSense\n");
        device = &dualsense;
    }
    device_updated = true;
}

double Joypad::last_recv_sec(){
    return (clock_->now() - joy.header.stamp).seconds();
}

bool Joypad::connected(){
    return last_recv_sec()<1.0;
}
