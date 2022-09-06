#include <iostream>
#include "connection/cmd_vel.h"

int main(int argc, char** argv)
{
    CMD_vel cmd;
    
    double speed;
    double R_SPEED;
    double wheel_dist;
    std::string port_name;

    ros::init(argc, argv, "connection");        // specify the name of node

	ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::NodeHandle n_private("~");

    
    n_private.param<double>("speed", speed, 4.0);
    n_private.param<double>("R_SPEED", R_SPEED, 0.5);
    n_private.param<double>("wheel_dist", wheel_dist, 0.5);
    n_private.param<std::string>("port_name", port_name, "/dev/wheel_motor");


    
    ros::Subscriber sub = n.subscribe("/cmd_vel/cmd_vel", 10, &CMD_vel::CMD_VEL, &cmd);     // YOUR JOB
    ros::Rate loop_rate(15);   

    while(ros::ok())
    {
        cmd.MOV(speed, R_SPEED, wheel_dist, port_name, nh);

        loop_rate.sleep();
        ros::spinOnce();  
        
    }

    return 0;
}