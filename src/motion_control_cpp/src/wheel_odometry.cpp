#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <string.h>
#include <math.h>
#include "tf2/LinearMath/Quaternion.h"

class WheelOdometry: public rclcpp::Node
{
public:
    WheelOdometry() : Node("wheel_odometry")
    {
        this->declare_parameter("wheel_radius", 0.0);
        this->declare_parameter("ticks_per_rev", 0);
        this->declare_parameter("refresh_rate", 0);
        this->declare_parameter("meters_per_tick", 0.0);
        this->declare_parameter("wheel_base", 0.0);
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("child_frame_id", "base_link");

        this->get_parameter("wheel_radius", wheel_radius);
        this->get_parameter("ticks_per_rev", ticks_per_rev);
        this->get_parameter("refresh_rate", refresh_rate);
        this->get_parameter("meters_per_tick", meters_per_tick);
        this->get_parameter("wheel_base", wheel_base);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("child_frame_id", child_frame_id);

        tick_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("/ticks", 10, std::bind(&WheelOdometry::set_ticks, this, std::placeholders::_1));
        wheel_odometry = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",10);
        pose_timer = this->create_wall_timer(std::chrono::milliseconds(refresh_rate), std::bind(&WheelOdometry::get_odom, this));

        RCLCPP_INFO(this->get_logger(), "Wheel Odometry node online.");

    }

    int right_ticks = 0;
    int left_ticks = 0;
    int old_right_ticks = 0;
    int old_left_ticks = 0;

    float heading = 0.0;
    float x = 0.0;
    float y = 0.0;

    float x_vel = 0.0;
    float y_vel = 0.0;
    float z_angular_vel = 0.0;

    void set_ticks(const std_msgs::msg::Int32MultiArray::SharedPtr ticks)
    {
        right_ticks = ticks->data[0];
        left_ticks = ticks->data[1];
    }


    void get_odom()
    {
        float d_x;
        float d_y;
        int d_right_ticks = right_ticks - old_right_ticks;
        int d_left_ticks = left_ticks - old_left_ticks;

        old_right_ticks = right_ticks;
        old_left_ticks = left_ticks;

        float circumference = wheel_radius*2*M_PI;
        float left_wheel_dist = (d_left_ticks / ticks_per_rev) * circumference;
        float right_wheel_dist = (d_right_ticks / ticks_per_rev) * circumference;
        float center_dist = (left_wheel_dist + right_wheel_dist) / 2;

        float d_heading = atan2(sin(heading + (right_wheel_dist - left_wheel_dist) / wheel_base), cos(heading + (right_wheel_dist - left_wheel_dist) / wheel_base));

        if (d_left_ticks != 0 && d_right_ticks != 0)
        {
            d_x = x + center_dist * cos(heading);
            d_y = y + center_dist * sin(heading);

            x = d_x;
            y = d_y;
            heading = d_heading;

            x_vel = d_x / refresh_rate;
            y_vel = d_y / refresh_rate;
            z_angular_vel = d_heading / refresh_rate;
        }
        else
        {
            x_vel = 0.0;
            y_vel = 0.0;
            z_angular_vel = 0.0;
        }

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header = std_msgs::msg::Header();
        rclcpp::Time now = this->get_clock()->now();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = frame_id;
        odom_msg.child_frame_id = child_frame_id;
        odom_msg.pose = geometry_msgs::msg::PoseWithCovariance();
        odom_msg.twist = geometry_msgs::msg::TwistWithCovariance();

        for (int i = 0; i < 36; i++)
        {
            if (i == 0 || i == 7 || i == 14)
            {
                odom_msg.pose.covariance[i] = 0.01;
                odom_msg.twist.covariance[i] = 0.01;
            }

            if (i == 21 || i == 28 || i ==35)
            {
                odom_msg.pose.covariance[i] += 0.01;
                odom_msg.twist.covariance[i] += 0.1;
            }
            else
            {
                odom_msg.pose.covariance[i] = 0.0;
                odom_msg.twist.covariance[i] = 0.0;
            }
        }

        odom_msg.pose.pose = geometry_msgs::msg::Pose();
        odom_msg.pose.pose.position = geometry_msgs::msg::Point();
        odom_msg.pose.pose.orientation = geometry_msgs::msg::Quaternion();

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = heading;
        
        float cy = cos(heading * 0.5);
        float sy = sin(heading * 0.5);
        float cp = cos(0 * 0.5);
        float sp = sin(0 * 0.5);
        float cr = cos(0 * 0.5);
        float sr = sin(0 * 0.5);

        odom_msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
        odom_msg.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
        odom_msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;
        odom_msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;

        odom_msg.twist.twist.linear.x = x_vel;
        odom_msg.twist.twist.linear.y = y_vel;
        odom_msg.twist.twist.linear.z = z_angular_vel;

        wheel_odometry->publish(odom_msg);
    }


private:

    float wheel_radius;
    int ticks_per_rev;
    int refresh_rate;
    float meters_per_tick;
    float wheel_base;
    std::string frame_id;
    std::string child_frame_id;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tick_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry;
    rclcpp::TimerBase::SharedPtr pose_timer;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<WheelOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}