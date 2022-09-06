#include <iostream>
#include "ros/ros.h"
#include <cmath>
#include <typeinfo>
#define _USE_MATH_DEFINES

class Location
{
    public:
        double x;
        double y;
        double th;
        double x_prev;
        double y_prev;
        double th_prev;
    Location(): x(0.0), y(0.0), th(0.0), x_prev(0.0), y_prev(0.0), th_prev(0.0)
    {

    };
    ~Location()
    {
    }
};

class Odom
{
    public:
        Location loc;
        int prev_R_pose;
        int prev_L_pose;
        double dist;
        double angle;

        void getodom(int R_pose, int L_pose, double wheel_dist, double dt)
        {
            double dist_per_pulse = 130 * (double) M_PI / 4096.0;  // dist_mm / pulse
            double x_dist, y_dist;
            double x_temp, y_temp, ang_temp;

            x_dist = ((double)R_pose - (double)prev_R_pose) * dist_per_pulse;
            y_dist = ((double)L_pose - (double)prev_L_pose) * dist_per_pulse;
            dist = (x_dist + y_dist) / 2.0;
            angle = (x_dist - y_dist) / wheel_dist;
            
            prev_R_pose = R_pose;
            prev_L_pose = L_pose;
            
            std::cout << R_pose << " " << L_pose << " " << std::endl;
            std::cout << x_dist << " " << y_dist << " " << angle << std::endl;
            std::cout << cos(angle) << std::endl;

            std::cout << "loc_prev " <<loc.x_prev << " " << loc.y_prev << " dist =" << dist << std::endl;
            
            x_temp = loc.x;
            loc.x_prev = x_temp;
            
            y_temp = loc.y;
            loc.y_prev = y_temp;

            ang_temp = loc.th;
            loc.th_prev = ang_temp;

            loc.x = x_temp + dist * cos(angle);
            loc.y = y_temp + dist * sin(angle);
            loc.th = ang_temp + angle;

            std::cout << "loc_e " <<loc.x << " " << loc.y << " " << loc.th << std::endl;

            loc.x_prev = loc.x;
            loc.y_prev = loc.y;
            loc.th_prev = loc.th;
        }

    Odom(): prev_R_pose(0), prev_L_pose(0), dist(0.0), angle(0.0)
    {
    }

    ~Odom()
    {
    }
};