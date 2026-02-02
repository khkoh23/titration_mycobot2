#ifndef CHECK_PH_BY_RANGE_CONDITION_H
#define CHECK_PH_BY_RANGE_CONDITION_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <chrono>

class CheckPhByRangeCondition : public BT::ConditionNode {
public:
    CheckPhByRangeCondition(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_(node_ptr) {
        RCLCPP_INFO(node_->get_logger(), "BT condition %s is started. Subscribing to /ph.", name.c_str());
        subscriber_ = node_->create_subscription<std_msgs::msg::Float32>("/ph", rclcpp::SensorDataQoS(), 
            std::bind(&CheckPhByRangeCondition::phCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("ph_min"),
            BT::InputPort<float>("ph_max"),
            BT::InputPort<std::string>("editable"),
        };
    }

    BT::NodeStatus tick() override {
        configureOnce(); // pulls ports the first time; safe to call repeatedly
        std::lock_guard<std::mutex> lock(mutex_);
        if (!last_ph_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
            "[%s] Waiting for /ph...", name().c_str());
            return BT::NodeStatus::RUNNING; //TODO FALSE?
        }
        const auto& ph = *last_ph_;
        // Basic sanity checks
        if (ph.data > 14.0 || ph.data < 0.0) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Received ph value %0.01f, but out of bound [0 to 14]!", name().c_str(), ph.data);
            return BT::NodeStatus::RUNNING; // If out of bound. Keep waiting until the pH value is within bound
        }
        // Logic
        if (ph.data >= ph_min_ && ph.data <= ph_max_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] pH value is within the range.", name().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "[%s] pH value is outside the range", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    std_msgs::msg::Float32::SharedPtr last_ph_;
    std::mutex mutex_; 

    void phCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_ph_ = msg;
    }

    float ph_min_{0.00};
    float ph_max_{14.00};
    bool configured_{false};

    void configureOnce() { // Read ports once (first tick) and cache
        if (configured_) return;
        (void)getInput<float>("ph_min", ph_min_);
        (void)getInput<float>("ph_max", ph_max_);
        configured_ = true;
    }

};

#endif // CHECK_PH_BY_RANGE_CONDITION_H