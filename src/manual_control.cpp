/**
* @file manual_control.cpp
* @brief manual control node
* @author Shunya Hara
* @date 2022.7.29
* @details 
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

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
    dynamixel_wrapper::dynamixel_wrapper motor0(0,dxl_base,dynamixel_wrapper::XM430,40.0);
    dynamixel_wrapper::dynamixel_wrapper motor1(1,dxl_base,dynamixel_wrapper::XM430,40.0);
    dynamixel_wrapper::dynamixel_wrapper motor2(2,dxl_base,dynamixel_wrapper::XM430,40.0);
    dynamixel_wrapper::dynamixel_wrapper motor3(3,dxl_base,dynamixel_wrapper::XM430,120.0);

    ros::NodeHandle lSubscriber("");
    ros::Subscriber joy_sub = lSubscriber.subscribe("/joy", 50, &joy_callback);
    joy_msg.axes.resize(20);
    joy_msg.buttons.resize(20);

    ros::Publisher pos_pub0=n.advertise<std_msgs::Float32>("motor0/angle", 10);
    ros::Publisher pos_pub1=n.advertise<std_msgs::Float32>("motor1/angle", 10);
    ros::Publisher pos_pub2=n.advertise<std_msgs::Float32>("motor2/angle", 10);
    ros::Publisher pos_pub3=n.advertise<std_msgs::Float32>("motor3/angle", 10);

    ros::Publisher current_pub0=n.advertise<std_msgs::Float32>("motor0/current", 10);
    ros::Publisher current_pub1=n.advertise<std_msgs::Float32>("motor1/current", 10);
    ros::Publisher current_pub2=n.advertise<std_msgs::Float32>("motor2/current", 10);
    ros::Publisher current_pub3=n.advertise<std_msgs::Float32>("motor3/current", 10);

    motor0.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);
    motor1.setTorqueEnable(false);

    std::vector<double> offset_angle={-53.964844, 81.474609, 287.314453, 20.214844};
    std::vector<double> max_angle={-200, 231.451172, 138.285156, 35.416016};

    while (n.ok())  {
        //motor3.setGoalPosition(200.0*(joy_msg.axes[0])+offset_angle[3]);
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

        //right
        if(joy_msg.axes[6]<0){
            motor0.setGoalPosition(max_angle[0]);
            motor1.setGoalPosition(offset_angle[1]);
            motor2.setGoalPosition(offset_angle[2]);
            motor3.setGoalPosition(max_angle[3]);
        }

        //up
        if(joy_msg.axes[7]>0){
            motor0.setGoalPosition(offset_angle[0]);
            motor1.setGoalPosition(max_angle[1]);
            motor2.setGoalPosition(offset_angle[2]);
            motor3.setGoalPosition(max_angle[3]);
        }

        //left
        if(joy_msg.axes[6]>0){
            motor0.setGoalPosition(offset_angle[0]);
            motor1.setGoalPosition(offset_angle[1]);
            motor2.setGoalPosition(max_angle[2]);
            motor3.setGoalPosition(max_angle[3]);
        }

        //down
        if(joy_msg.axes[7]<0){
            motor0.setGoalPosition(offset_angle[0]);
            motor1.setGoalPosition(offset_angle[1]);
            motor2.setGoalPosition(offset_angle[2]);
            motor3.setGoalPosition(offset_angle[3]);
        }

        /*
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
        */

        std::vector<std_msgs::Float32> pos_msg(4);
        pos_msg[0].data=motor0.getCurrentPosition()-offset_angle[0];
        pos_msg[1].data=motor1.getCurrentPosition()-offset_angle[1];
        pos_msg[2].data=motor2.getCurrentPosition()-offset_angle[2];
        pos_msg[3].data=motor3.getCurrentPosition()-offset_angle[3];
        pos_pub0.publish(pos_msg[0]);
        pos_pub1.publish(pos_msg[1]);
        pos_pub2.publish(pos_msg[2]);
        pos_pub3.publish(pos_msg[3]);

        std::vector<std_msgs::Float32> current_msg(4);
        current_msg[0].data=motor0.getCurrentCurrent();
        current_msg[1].data=motor1.getCurrentCurrent();
        current_msg[2].data=motor2.getCurrentCurrent();
        current_msg[3].data=motor3.getCurrentCurrent();
        current_pub0.publish(current_msg[0]);
        current_pub1.publish(current_msg[1]);
        current_pub2.publish(current_msg[2]);
        current_pub3.publish(current_msg[3]);

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}