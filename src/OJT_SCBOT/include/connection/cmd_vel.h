#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>
#include "connection/connection.h"


//#define speed 0.25
//#define T_RPM 148
// #define speed 1

#define T_RPM 146


class CMD_vel
{
    public:
        
        int run_time = 0;
        float T_vel = 0.0f;
        float R_vel = 0.0f;
        int rpm_1 = 0;
        int rpm_2 = 0;
        CONNECTION con;

        void CMD_VEL(const geometry_msgs::Twist::ConstPtr& msg)
        {
            T_vel = msg->linear.x;
            R_vel = msg->angular.z;
        }
        
        void MOV(double speed, double R_SPEED, double wheel_dist, std::string port_name, ros::NodeHandle nh)
        {
            rpm_1 = speed * T_RPM * (T_vel + R_vel*1.72*R_SPEED);   // Don't modify
            rpm_2 = speed * T_RPM * (T_vel - R_vel*1.72*R_SPEED);   // Don't modify

            con.receive_task(wheel_dist);   // Don't modify
            con.send_task(rpm_1, rpm_2);    // Don't modify
            con.publish(nh);                // Don't modify

            if(run_time == 0)
            {
                run_time = con.CONNECT(port_name);
            }
        }
};
