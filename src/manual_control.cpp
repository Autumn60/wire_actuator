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
    double rate=10.0;
    //制御周期10Hz
    ros::Rate loop_rate(rate);

    //param setting
    ros::NodeHandle pn("~");
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;

    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(0,dxl_base,dynamixel_wrapper::XM430,50.0);
    dynamixel_wrapper::dynamixel_wrapper motor1(1,dxl_base,dynamixel_wrapper::XM430,50.0);
    dynamixel_wrapper::dynamixel_wrapper motor2(2,dxl_base,dynamixel_wrapper::XM430,100.0);
    dynamixel_wrapper::dynamixel_wrapper motor3(3,dxl_base,dynamixel_wrapper::XM430,120.0);

    ros::NodeHandle lSubscriber("");
    ros::Subscriber joy_sub = lSubscriber.subscribe("/joy", 50, &joy_callback);
    joy_msg.axes.resize(20);
    joy_msg.buttons.resize(20);

    motor0.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor1.setTorqueEnable(false);

    std::vector<double> offset_angle={295.585938, 101.451172, 272.285156, 51.416016};

    while (n.ok())  {
        motor3.setGoalPosition(200.0*(joy_msg.axes[0])+offset_angle[3]);
        //motor1.write(11,1,0);
        ROS_INFO("Positions:{%lf, %lf, %lf, %lf}",motor0.getCurrentPosition(),motor1.getCurrentPosition(),motor2.getCurrentPosition(),motor3.getCurrentPosition());

        //circle
        if(joy_msg.buttons[1]){
            motor0.setTorqueEnable(true);
            motor1.setTorqueEnable(true);
            motor2.setTorqueEnable(true);
            motor3.setTorqueEnable(true);
        }
        //closs
        if(joy_msg.buttons[0]){
            motor0.setTorqueEnable(false);
            motor1.setTorqueEnable(false);
            motor2.setTorqueEnable(false);
            motor3.setTorqueEnable(false);
        }

        const double duration=1/rate;
        const double amplitude=90.0;
        const double hz=1.0;
        static double current_angle=0.0;
        if(joy_msg.buttons[2]){
            current_angle+=hz*duration*2.0*M_PI;
            motor0.setGoalPosition(-offset_angle[0]+amplitude+amplitude*sin(current_angle));
            motor1.setGoalPosition(offset_angle[1]+amplitude+amplitude*sin(current_angle+2.0/3.0*M_PI));
            motor2.setGoalPosition(-offset_angle[2]+amplitude+amplitude*sin(current_angle+4.0/3.0*M_PI));
        }
        if(false){
            motor0.setGoalPosition(offset_angle[0]+amplitude+amplitude*sin(current_angle));
            motor1.setGoalPosition(offset_angle[1]+amplitude+amplitude*sin(current_angle+2.0/3.0*M_PI));
            motor2.setGoalPosition(offset_angle[2]+amplitude+amplitude*sin(current_angle+4.0/3.0*M_PI));
        }
        
        //ROS_INFO("current:%d",motor1.read(64,1));
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}