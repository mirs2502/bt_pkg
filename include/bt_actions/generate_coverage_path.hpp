#ifndef GENERATE_COVERAGE_PATH_HPP
#define GENERATE_COVERAGE_PATH_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace BT;

class GenerateCoveragePath : public SyncActionNode
{
public:
    GenerateCoveragePath(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort<rclcpp::Node::SharedPtr>("node"),
            InputPort<std::vector<geometry_msgs::msg::Point>>("area_polygon"),
            OutputPort<nav_msgs::msg::Path>("coverage_path")
        };
    }

    NodeStatus tick() override {
        auto node_res = getInput<rclcpp::Node::SharedPtr>("node");
        if (!node_res) {
             throw RuntimeError("Missing required input [node]");
        }
        auto node = node_res.value();
        
        auto polygon_res = getInput<std::vector<geometry_msgs::msg::Point>>("area_polygon");
        if (!polygon_res) {
            RCLCPP_ERROR(node->get_logger(), "Missing input [area_polygon]");
            return NodeStatus::FAILURE;
        }
        auto polygon = polygon_res.value();

        if (polygon.size() < 3) {
            RCLCPP_ERROR(node->get_logger(), "Polygon has fewer than 3 points.");
            return NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node->get_logger(), ">> [Phase 2] Generating coverage path for polygon with %zu points...", polygon.size());

        // 1. Calculate Bounding Box
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto& p : polygon) {
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }

        // 2. Generate Zigzag Path
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = node->now();

        double step_y = 0.3; // Robot width / overlap
        bool direction = true; // true: left to right, false: right to left

        for (double y = min_y + step_y / 2.0; y <= max_y; y += step_y) {
            std::vector<double> intersections;
            
            // Find intersections with all edges
            for (size_t i = 0; i < polygon.size(); ++i) {
                auto p1 = polygon[i];
                auto p2 = polygon[(i + 1) % polygon.size()];

                if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) {
                    double x = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
                    intersections.push_back(x);
                }
            }

            std::sort(intersections.begin(), intersections.end());

            // Add segments
            if (direction) {
                for (size_t i = 0; i < intersections.size(); i += 2) {
                    if (i + 1 < intersections.size()) {
                        addPose(path_msg, intersections[i], y);
                        addPose(path_msg, intersections[i+1], y);
                    }
                }
            } else {
                for (int i = intersections.size() - 1; i >= 0; i -= 2) {
                    if (i - 1 >= 0) {
                        addPose(path_msg, intersections[i], y);
                        addPose(path_msg, intersections[i-1], y);
                    }
                }
            }
            direction = !direction;
        }

        RCLCPP_INFO(node->get_logger(), "Generated path with %zu waypoints.", path_msg.poses.size());
        setOutput("coverage_path", path_msg);

        return NodeStatus::SUCCESS;
    }

private:
    void addPose(nav_msgs::msg::Path& path_msg, double x, double y) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0; // Default orientation
        path_msg.poses.push_back(pose);
    }
};
#endif