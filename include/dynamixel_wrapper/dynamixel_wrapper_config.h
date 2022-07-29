/**
* @file dynamixel_wrapper_config.h
* @brief easy use for dynamixel_sdk
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#pragma once

#include <ros/ros.h>
#include <string>
#include <dynamixel_sdk/dynamixel_sdk.h>

class dynamixel_wrapper_config{
    public:
    const int torque_enable;
    const int torque_enable_size;

    
};