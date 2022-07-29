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

namespace dynamixel_wrapper{

class dynamixel_wrapper_config{
    public:
    int torque_enable;
    int torque_enable_size;
    int current_limit;
    int current_limit_size;
    double current_scaling_factor;
    int goal_position;
    int goal_position_size;
    int goal_velocity;
    int goal_velocity_size;
    int goal_current;
    int goal_current_size;
    
};

dynamixel_wrapper_config XM430{
    //address,byte_size
    //torque
    64,1,
    //current_limit
    38,2,2.69,
    //goal position
    116,4,
    //goal velocity
    104,4,
    //goal_current
    102,2

};

}//namespace dynamixel_wrapper