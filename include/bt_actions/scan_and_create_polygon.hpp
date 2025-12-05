#ifndef SCAN_AND_CREATE_POLYGON_HPP
#define SCAN_AND_CREATE_POLYGON_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace BT;

class ScanAndCreatePolygon : public StatefulActionNode
{
public:
    ScanAndCreatePolygon(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            OutputPort<std::vector<geometry_msgs::msg::Point>>("area_polygon")
        };
    }

    NodeStatus onStart() override {
        auto node_res = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_res) {
            throw RuntimeError("Missing required input [node]");
        }
        node_ = node_res.value();

        // Subscribe to /cone_area
        // Note: We use a lambda that captures 'this' to update the member variable.
        // Since the node spins in the main loop, this callback will be executed there.
        sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/cone_area", 10,
            [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
                last_polygon_ = msg;
            });

        RCLCPP_INFO(node_->get_logger(), "Waiting for polygon from /cone_area...");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        if (last_polygon_) {
            std::vector<geometry_msgs::msg::Point> points;
            for (const auto& p32 : last_polygon_->polygon.points) {
                geometry_msgs::msg::Point p;
                p.x = p32.x;
                p.y = p32.y;
                p.z = p32.z;
                points.push_back(p);
            }

            setOutput("area_polygon", points);
            RCLCPP_INFO(node_->get_logger(), "Polygon received with %zu points.", points.size());
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        sub_.reset();
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_;
    geometry_msgs::msg::PolygonStamped::SharedPtr last_polygon_;
};

#endif