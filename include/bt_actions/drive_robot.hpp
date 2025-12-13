#ifndef DRIVE_ROBOT_HPP
#define DRIVE_ROBOT_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace BT;

class DriveRobot : public AsyncActionNode
{
public:
    DriveRobot(const std::string& name, const NodeConfiguration& config)
        : AsyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<double>("linear_x"),
            InputPort<double>("angular_z"),
            InputPort<double>("duration")
        };
    }

    NodeStatus tick() override {
        auto node_res = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_res) {
             throw RuntimeError("Missing required input [node]");
        }
        auto node = node_res.value();

        double linear_x = 0.0;
        double angular_z = 0.0;
        double duration = 0.0;

        getInput("linear_x", linear_x);
        getInput("angular_z", angular_z);
        if (!getInput("duration", duration)) {
            throw RuntimeError("Missing required input [duration]");
        }

        RCLCPP_INFO(node->get_logger(), "DriveRobot: Driving (v=%.2f, w=%.2f) for %.1f sec", linear_x, angular_z, duration);

        auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        
        rclcpp::Rate rate(10); // 10Hz

        while (std::chrono::steady_clock::now() < end_time) {
            if (isHalted()) {
                return NodeStatus::IDLE;
            }

            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_x;
            msg.angular.z = angular_z;
            publisher->publish(msg);

            rate.sleep();
        }

        // Stop the robot at the end
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        publisher->publish(stop_msg);

        RCLCPP_INFO(node->get_logger(), "DriveRobot: Finished");
        return NodeStatus::SUCCESS;
    }
    
    void halt() override {
        AsyncActionNode::halt();
    }
};

#endif
