#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <string.h>

class DifferentialDriveNode : public rclcpp::Node
{
public:
    DifferentialDriveNode() : Node("differential_drive")
    {
        this->declare_parameter("wheel_base", 0.0);
        this->declare_parameter("wheel_radius", 0.0);
        this->declare_parameter("max_linear_vel", 0.0);
        this->declare_parameter("max_angular_vel", 0.0);

        this->get_parameter("wheel_base", wheel_base);
        this->get_parameter("wheel_radius", wheel_radius);
        this->get_parameter("max_linear_vel", max_linear_vel);
        this->get_parameter("max_angular_vel", max_angular_vel);

        target_vels = this->create_publisher<std_msgs::msg::Float32MultiArray>("/target_velocities", 10);
        commands = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&DifferentialDriveNode::calculateWheelVelocities,this,std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Differential Drive node online.");
        RCLCPP_INFO(this->get_logger(), ("Wheel base: " + std::to_string(wheel_base)));
    }

private:
    double wheel_base;
    double wheel_radius;
    double max_linear_vel;
    double max_angular_vel;

    void calculateWheelVelocities(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        float linear_vel = cmd->linear.x;
        float angular_vel = cmd->angular.z;

        float vr = linear_vel + ((angular_vel * wheel_base) / 2);
        float vl = linear_vel - ((angular_vel * wheel_base) / 2);

        auto wheel_vels = std_msgs::msg::Float32MultiArray();
        wheel_vels.data = {vr, vl};
        target_vels->publish(wheel_vels);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_vels;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
