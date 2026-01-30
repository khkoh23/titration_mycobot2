#ifndef SET_DISPENSE_VOLUME_ACTION_H
#define SET_DISPENSE_VOLUME_ACTION_H

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <titration_robot_interfaces/srv/syringe.hpp>
#include <memory>
#include <chrono>
#include <future>

class SetDispenseVolumeAction : public BT::StatefulActionNode {
public:
    using Syringe = titration_robot_interfaces::srv::Syringe;

    SetDispenseVolumeAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
        bt_node_name_ = name;
        client_ = node_->create_client<Syringe>("syringe");
    }

    BT::NodeStatus onStart() override {
        if (!getInput<unsigned>(std::string("volume"), volume_)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input ports: volume", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        using namespace std::chrono_literals;
        RCLCPP_INFO(node_->get_logger(), "BT action %s is started.", bt_node_name_.c_str());
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Service server 'syringe' is not available!", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto req_msg = std::make_shared<Syringe::Request>(); 
        req_msg->command = 91;
        req_msg->dispense_volume = volume_;
        auto far = client_->async_send_request(req_msg); 
        future_ = far;
        start_time_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        using namespace std::chrono_literals;
        if (!rclcpp::ok()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] ROS shutting down.", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        // Non-blocking poll on shared_future
        if (future_.valid() && future_.wait_for(0s) == std::future_status::ready) { 
            auto resp = future_.get();  // Response::SharedPtr
            // TODO: check any fields in resp to decide success/failure if the service defines them
            RCLCPP_INFO(node_->get_logger(), "[%s] Service request for syringe completed.", bt_node_name_.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        if (elapsed > 2s) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Service request for syringe timed out after 2s.", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<unsigned>(std::string("volume")),
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Syringe>::SharedPtr client_;
    rclcpp::Client<Syringe>::SharedFuture future_;
    std::chrono::steady_clock::time_point start_time_{};
    std::string bt_node_name_;

    unsigned volume_{200};

}; 

#endif // SET_DISPENSE_VOLUME_ACTION_H