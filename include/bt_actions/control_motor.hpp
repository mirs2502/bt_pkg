#ifndef CONTROL_MOTOR_HPP
#define CONTROL_MOTOR_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "mirs_msgs/srv/basic_command.hpp"

using namespace BT;

class ControlMotor : public AsyncActionNode
{
public:
    ControlMotor(const std::string& name, const NodeConfiguration& config)
        : AsyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<int>("pwm_value")
        };
    }

    NodeStatus tick() override {
        auto node_res = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_res) {
             throw RuntimeError("Missing required input [node]");
        }
        auto node = node_res.value();

        auto pwm_res = getInput<int>("pwm_value");
        if (!pwm_res) {
            RCLCPP_ERROR(node->get_logger(), "Missing input [pwm_value]");
            return NodeStatus::FAILURE;
        }
        int pwm_value = pwm_res.value();

        RCLCPP_INFO(node->get_logger(), "ControlMotor: Calling service with PWM %d", pwm_value);

        auto client = node->create_client<mirs_msgs::srv::BasicCommand>("/motor_ctrl");

        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node->get_logger(), "Service /motor_ctrl not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<mirs_msgs::srv::BasicCommand::Request>();
        request->param1 = (double)pwm_value;

        auto future = client->async_send_request(request);

        // Wait for the result
        // Since we are in AsyncActionNode, we are in a separate thread.
        // We can block here waiting for the future, while the main thread spins the node.
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
            RCLCPP_ERROR(node->get_logger(), "Service call timed out");
            return NodeStatus::FAILURE;
        }

        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(node->get_logger(), "ControlMotor: Service call successful");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node->get_logger(), "ControlMotor: Service returned failure");
            return NodeStatus::FAILURE;
        }
    }
};

#endif
