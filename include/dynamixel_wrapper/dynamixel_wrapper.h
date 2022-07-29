/**
* @file dynamixel_wrapper.h
* @brief easy use for dynamixel_sdk
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#pragma once

#include <ros/ros.h>
#include <string>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper_base.h>
#include <dynamixel_wrapper/dynamixel_wrapper_config.h>
namespace dynamixel_wrapper{

class dynamixel_wrapper{
    public:
    dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config);
    void write(int address,int byte_size, int value);
    uint32_t read(int address,int byte_size);
    void setPosition(double rad){write(motor_config_.goal_position,motor_config_.goal_position_size,rad);}
    void setCurrentLimit(double current/*[mA]*/){write(motor_config_.current_limit,motor_config_.current_limit_size,current/motor_config_.current_scaling_factor);}

    private:
    int id_;
    dynamixel_wrapper_base* dxl_base_;
    dynamixel_wrapper_config motor_config_;
};




dynamixel_wrapper::dynamixel_wrapper(const int& id, dynamixel_wrapper_base& dxl_base, const dynamixel_wrapper_config& motor_config){
    id_=id;
    dxl_base_= &dxl_base;
    motor_config_=motor_config;
    write(11,1,5);
    bool dxl_comm_result = dxl_base_->packetHandler->write1ByteTxRx(
    dxl_base_->portHandler, id_, 64, 1);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", id_);
    }
}

void dynamixel_wrapper::write(int address,int byte_size, int value){
    bool dxl_comm_result;
    if(byte_size==1){
        dxl_comm_result = dxl_base_->packetHandler->write1ByteTxRx(dxl_base_->portHandler, id_, address, value);
    }
    else if(byte_size==2){
        dxl_comm_result = dxl_base_->packetHandler->write2ByteTxRx(dxl_base_->portHandler, id_, address, value);
    }
    else if(byte_size==4){
        dxl_comm_result = dxl_base_->packetHandler->write4ByteTxRx(dxl_base_->portHandler, id_, address, value);
    }
    else{
        ROS_ERROR("Byte size is undefined");
    }
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", id_);
    }
}

uint32_t dynamixel_wrapper::read(int address,int byte_size){
    uint8_t value8;
    uint16_t value16;
    uint32_t value32;
    if(byte_size==1){
        dxl_base_->packetHandler->read1ByteTxRx(dxl_base_->portHandler, id_, address, &value8);
        value32=value8;
    }
    else if(byte_size==2){
        dxl_base_->packetHandler->read2ByteTxRx(dxl_base_->portHandler, id_, address, &value16);
        value32=value16;
    }
    else if(byte_size==4){
        dxl_base_->packetHandler->read4ByteTxRx(dxl_base_->portHandler, id_, address, &value32);
    }
    else{
        ROS_ERROR("Byte size is undefined");
        return 0;
    }
    
    return value32;
}

}//namespace dynamixel_wrapper