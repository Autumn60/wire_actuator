/**
* @file manual_control.cpp
* @brief manual control node
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>


int main(int argc, char **argv){
    
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //param setting
    ros::NodeHandle pn("~");
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;

    dynamixel_wrapper_base dxl_base(port_name,baudrate);
    dynamixel_wrapper motor1(1,dxl_base);


    while (n.ok())  {
        motor1.write(64,1,0);
        ROS_INFO("spin%d",motor1.read(64,1));
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}