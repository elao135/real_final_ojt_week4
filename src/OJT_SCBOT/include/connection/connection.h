#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <mutex>
#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include "sensor_msgs/Joy.h"
#include <iostream>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// #include "connection/odom.h"
// #include "connection/joy.h"
#define T_RPM 146

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

class PORT
{
    public:
        // std::string port_name = "/dev/ttyUSB0";
        std::string baudrate = "19200";
};

class CONNECTION
{
    private:
        struct MOTOR_INFO
        {
            short L_rpm;
            short L_current;
            char L_state;
            int L_pose;

            short R_rpm;
            short R_current;
            char R_state;
            int R_pose;
        };

        MOTOR_INFO prev_mi;
        Location loc;
        // void pub_odom(Location location);
    public:
        PORT port;
        int fd;
        bool tqoff;
        int read_buf_pos = 0;
        int write_buf_pos = 0;
        unsigned char read_buf[256];
        unsigned char write_buf[256];
        std::mutex fd_mtx;
        double prev_read_time;
        // ros::Publisher pub_odom;
        // std::chrono::system_clock::time_point prev_read_time;
        int val = 0;
        // int t_rpm = 100;

        // Odom odom;

        ros::Publisher pub_odom;

    void publish(ros::NodeHandle nh)
    {
        pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 100);
    }

    bool CONNECT(std::string port_name)
    {
        fd = 0;
        struct termios newtio;

        while(ros::ok())
        {
            fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);

            if(fd < 0)
            {
                ROS_ERROR("Not connected, check your USB port.");
            }
            else
                break;
        }

        ROS_INFO("RObot connected!", port_name.c_str());

        memset(&newtio, 0, sizeof(newtio));

        newtio.c_cflag = B19200;
        newtio.c_cflag |= CS8;
        newtio.c_cflag |= CLOCAL;
        newtio.c_cflag |= CREAD;
        newtio.c_iflag = 0;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;
        newtio.c_cflag |= CS8;

        tcflush(fd, TCIFLUSH);

        tcsetattr(fd, TCSANOW, &newtio);

        return 1;
    }

    bool write_serial(unsigned char* data, int size)
    {
        // ROS_INFO("write_serial");

        if(size == 0 | fd <= 0) return 0;
        bool check_write = false;

        // fd_mtx.lock();
        
        val = write(fd, data, size);
        
        if(val == size)
            check_write = true;

        // fd_mtx.unlock();

        return check_write;
    }

    void send_task(float T_vel, float R_vel)
    {
        // ROS_INFO("send_task");
        run_motor(T_vel, R_vel);
    }

    short run_motor(int rpm_1, int rpm_2)
    {
        unsigned char l_rpm = 0;
        unsigned char r_rpm = 0;

        if(fd <= 0)
        {
            ROS_WARN("write2motor fd: %d", fd);

            return 0;
        }

        unsigned char write_motor_cmd[13] = {183, 184, 1, 207, 7, 1, 0, 0, 1, 0, 0, 0, 184};

        // ROS_INFO("joy_FB: %lf", joy_ACT); 

        ROS_INFO("cmd= rpm_1: %d, rpm_2: %d", -rpm_1, rpm_2);
            
        if(rpm_1 > 0)
        {
            r_rpm = rpm_1;
            write_motor_cmd[9] = r_rpm;
            write_motor_cmd[10] = 0;
        }
        else if(rpm_1 < 0)
            {
            r_rpm = 256 - abs(rpm_1);
            write_motor_cmd[9] = r_rpm;
            write_motor_cmd[10] = 255;
        }
        if(rpm_2 > 0)
        {
            l_rpm = 256 - rpm_2;
            write_motor_cmd[6] = l_rpm;
            write_motor_cmd[7] = 255;
        }
        else if(rpm_2 < 0)
        {
            l_rpm = abs(rpm_2);
            write_motor_cmd[6] = l_rpm;
            write_motor_cmd[7] = 0;
        }


        unsigned char chk = check_sum_send(&write_motor_cmd[0],12);
        write_motor_cmd[12] = chk;

        // ROS_INFO("left_wheel_val: %d", write_motor_cmd[6]);

        write_speed_serial(write_motor_cmd, 13);
    }

    bool write_speed_serial(unsigned char* data, int size)
    {
        if(size == 0 | fd <= 0)
        return 0;

        bool check_write = false;

        // fd_mtx.lock();
        int val = write(fd, data, size);
        if(val == size)
        {
            check_write = true;
        }   
        // fd_mtx.unlock();

        return check_write;
    }

    void receive_task(double wheel_dist)
    {
        // ROS_INFO("receive task");
            read_motor_state(wheel_dist);
    }

    short read_motor_state(double wheel_dist)
    {

        double dist_per_pulse = 0.130 * (double) M_PI / 60.0;  // dist_mm / pulse

        // ROS_INFO("read motor speed");
        if(fd <= 0)
        {
            ROS_WARN("read_motor_state fd: %d", fd);
            
            return 0;
        }
        
        request_motor_state();

        double read_time = ros::Time::now().toSec();
        double dt_duration = read_time - prev_read_time;
        // std::chrono::system_clock::time_point read_time = std::chrono::system_clock::now();
        // std::chrono::duration<double> dt_duration = read_time - prev_read_time;
        
        prev_read_time = read_time;

        // double dt = dt_duration.count();

        double dt = dt_duration;

        int read_size = read(fd, &read_buf[read_buf_pos], 128);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // ROS_INFO("read_size: %d", read_size);
        // ROS_INFO("fd: %d", fd);
    
        read_buf_pos += read_size;


        // ROS_INFO("read_buf_pos: %d", read_buf_pos);
        

        if(read_size <= 0)
            return 0;

        unsigned char read_id[5] = {184, 183, 1, 210, 18};

        for(int i = 0; i < read_buf_pos - 23; i++)
        {
            if(read_buf[i] == read_id[0])
            if(read_buf[i+1] == read_id[1])
            if(read_buf[i+2] == read_id[2])
            if(read_buf[i+3] == read_id[3])
            if(read_buf[i+4] == read_id[4])
            {
                MOTOR_INFO mi;

                int R_sign = -1, L_sign = -1;

                mi.R_rpm = (short)R_sign * Byte2Short(read_buf[i+5], read_buf[i+6]);
                mi.L_rpm = (short)L_sign * Byte2Short(read_buf[i+14], read_buf[i+15]);

                mi.L_current = Byte2Short(read_buf[i+7], read_buf[i+8]);
                mi.R_current = Byte2Short(read_buf[i+16], read_buf[i+17]);

                mi.L_state = read_buf[i+9];
                mi.R_state = read_buf[i+18];

                mi.R_pose = -Byte2int(read_buf[i+10], read_buf[i+11], read_buf[i+12], read_buf[i+13]);
                mi.L_pose = Byte2int(read_buf[i+19], read_buf[i+20], read_buf[i+21], read_buf[i+22]);

                unsigned char chk = check_sum_send(&read_buf[i], 23);
                
                if(read_buf[i+23] != chk)
                {
                    ROS_WARN("checksum error");

                    continue;
                }
                int remain_size = read_buf_pos - i - 24;
                unsigned char temp[256] = {};
                memcpy(temp, &read_buf[i+24],remain_size);
                memcpy(read_buf, temp, remain_size);

                i = -1;
                read_buf_pos = remain_size;

                int L_d_pose = mi.L_pose - prev_mi.L_pose;
                int R_d_pose = mi.R_pose - prev_mi.R_pose;

                memcpy(&prev_mi, &mi, sizeof(MOTOR_INFO));

                if(abs(L_d_pose) > 100 || abs(R_d_pose) > 100)
                {
                    ROS_WARN("counter error R: %d, L: %d", R_d_pose, L_d_pose);
                    continue;
                }        
                
                double L_d_distance = (double)L_d_pose * dist_per_pulse;
                double R_d_distance = (double)R_d_pose * dist_per_pulse;

                double distance = (L_d_distance + R_d_distance) / 2.0;
                double radian = -(R_d_distance - L_d_distance) / wheel_dist;
                double angle = radian/(double)M_PI*180.0;

                loc.x = loc.x_prev + distance * cos(loc.th);
                loc.y = loc.y_prev + distance * sin(loc.th);
                loc.th = loc.th_prev + radian;

                loc.x_prev = loc.x;
                loc.y_prev = loc.y;
                loc.th_prev = loc.th;

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(loc.th);

                nav_msgs::Odometry odom;

                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "/odom";

                odom.pose.pose.position.x = loc.x;
                odom.pose.pose.position.y = loc.y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                odom.child_frame_id = "/base_link";

                pub_odom.publish(odom);
            }   
        }
    }

    short Byte2Short(unsigned char low, unsigned char high)
    {
        return ((short) low | (short) high << 8);
    }

    int Byte2int(unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4)
    {
        return ((int) d1 | (int) d2 << 8 | (int) d3 << 16 | (int) d4 << 24);
    }

    void request_motor_state()
    {
    
        // ROS_INFO("request motor state");
        unsigned char read_motor_cmd[7] = {183, 184, 1, 4, 1, 210, 185};
        unsigned char chk = check_sum_send(&read_motor_cmd[0], 6);
        // ROS_INFO("chk: %i", chk);
        write_serial(read_motor_cmd,7);
        
    }

    unsigned char check_sum_send(unsigned char* Array, int size)
    {
        int byTmp = 0;
        short i;

        for(i = 0; i < size; i++)
        {
            byTmp += *(Array + i);
        }

        return (~byTmp + 1);
    }

    unsigned char check_sum_receive(unsigned char* Array, int size)
    {
        short i;
        int cbySum = 0;

        for(i = 0; i < size; i++)
        {
            cbySum += *(Array + i);
        }
        if(cbySum == 0)
            return 1;
        else
            return 0;
    }
};