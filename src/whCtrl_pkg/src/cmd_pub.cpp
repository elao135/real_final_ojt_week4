#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

ros::Publisher vel_pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = joy->axes[0];
    twist.linear.x = joy->axes[1];
    if(joy->buttons[6]==1)
    {
        vel_pub.publish(twist);
    }
}

int main(int argc, char ** argv)
{
    ros::init (argc, argv, "cmd_pub");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel/cmd_vel",1);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy",10,joyCallback);
    ros::spin();
}