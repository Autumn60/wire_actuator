/**
* @file manual_control.cpp
* @brief manual control node
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

sensor_msgs::Joy joy_msg;
void joy_callback(const sensor_msgs::Joy& msg){
    joy_msg=msg;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //param setting
    ros::NodeHandle pn("~");
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;

    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor1(1,dxl_base,dynamixel_wrapper::XM430);

    ros::NodeHandle lSubscriber("");

    //Path subscliber
    
    ros::Subscriber joy_sub = lSubscriber.subscribe("/joy", 50, &joy_callback);
    joy_msg.axes.resize(20);

    while (n.ok())  {
        motor1.setPosition(2000*(joy_msg.axes[0]+1));
        motor1.setCurrentLimit(10);
        motor1.write(11,1,0);
        ROS_INFO("%d",motor1.read(11,1));
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}