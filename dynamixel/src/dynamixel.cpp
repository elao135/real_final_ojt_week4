#include"ros/ros.h"
#include"dynamixel_workbench_msgs/DynamixelCommand.h"
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

class Joy
{
public:
    ros::NodeHandle nh;
    ros::Publisher joy_pub;
    //geometry_msgs::Twist twist;

    ros::ServiceClient client;

    dynamixel_workbench_msgs::DynamixelCommand srv;
    std::string servo_command = "";
    std::string servo_addrname = "Goal_Position";

    int now_value = 0; //dynamixel -> int
    int turn_set = 50;
    int next_value = 0;
    bool trigger; 
    float joy_row;
    float joy_col;
    // Args: command id addr_name value

    void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        trigger = joy->buttons[7];
        joy_col = joy->axes[3];    //twist.linear.x = joy->axes[3]; 
        joy_row = joy->axes[2];
        if(trigger == 1.0)
        {
            if(joy_row != 0.0) //class
            {
                next_value = now_value + joy_col*turn_set;
                
                if(0<=next_value && next_value<=1023)
                { 
                    srv.request.value = next_value;
                }
                else if(next_value<=0)
                {
                    next_value = 0.0;
                    srv.request.value = next_value;
                }
                else if(next_value>=1023)
                {
                    next_value = 1023;
                    srv.request.value = next_value;
                }
                now_value = next_value; //now_value update
                client.call(srv);
            } 
        }
    }

Joy()
{   
    srv.request.value = 0;
    client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    srv.request.command = servo_command;
    srv.request.id = 5;
    srv.request.addr_name = servo_addrname;
}
~Joy()
{
}

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_srv");
    ros::NodeHandle nh;
    Joy joy;

    ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>("/joy",10, &Joy::JoyCallback, &joy);
    ros::spin();

}