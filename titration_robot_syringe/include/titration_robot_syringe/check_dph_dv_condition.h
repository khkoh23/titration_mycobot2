#ifndef CHECK_DPH_DV_CONDITION_H
#define CHECK_DPH_DV_CONDITION_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <titration_robot_interfaces/srv/derivative_ph_volume.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>     // for std::this_thread::sleep_for
#include <rmw/rmw.h>  // rmw_request_id_t

class CheckDphDvCondition : public BT::ConditionNode {
public:
    using DerivativePhVolume = titration_robot_interfaces::srv::DerivativePhVolume;
    CheckDphDvCondition(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::ConditionNode(name, config), node_(std::move(node_ptr)), bt_node_name_(name) {
        client_ = node_->create_client<DerivativePhVolume>("derivative_ph_volume");
        RCLCPP_INFO(node_->get_logger(), "BT condition %s started.", bt_node_name_.c_str());
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>(std::string("criteria_min")),
            BT::InputPort<float>(std::string("criteria_max")),
        }; 
    }

    BT::NodeStatus tick() override {
        configureOnce(); // pulls ports the first time; safe to call repeatedly
        using namespace std::chrono_literals;
        if (!client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[%s] Service server 'derivative_ph_volume' not available!", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto req = std::make_shared<DerivativePhVolume::Request>();
        (void)client_->async_send_request(req);
        // Poll for the response with a bounded wait.
        constexpr auto kPollSleep = 10ms;      // poll granularity
        constexpr auto kMaxWait   = 100ms;     // total max wait per tick (tune this)
        const auto t_start = std::chrono::steady_clock::now();
        DerivativePhVolume::Response resp_msg;
        rmw_request_id_t resp_header;
        while (true) {
            // Attempt to take a response; returns true if one was available.
            if (client_->take_response(resp_msg, resp_header)) {
                const float hv = static_cast<float>(resp_msg.dph_dv);
                RCLCPP_DEBUG(node_->get_logger(), "[%s] Fresh server response: dph_dv=%.6f.", bt_node_name_.c_str(), hv);
                // Logic
                if (abs(hv) >= criteria_min_ && abs(hv) < criteria_max_) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] Derivative is within the criteria.", name().c_str());
                    return BT::NodeStatus::SUCCESS;
                }
                else {
                    RCLCPP_WARN(node_->get_logger(), "[%s] Derivative is beyond the criteria.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }                
            }
            // No response yet; bail out if we exceeded max wait
            if (std::chrono::steady_clock::now() - t_start >= kMaxWait) {
                RCLCPP_WARN(node_->get_logger(), "[%s] No response within %ld ms; treating as FAILURE this tick.", bt_node_name_.c_str(), 
                std::chrono::duration_cast<std::chrono::milliseconds>(kMaxWait).count());
                return BT::NodeStatus::FAILURE;
            }
            std::this_thread::sleep_for(kPollSleep);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<DerivativePhVolume>::SharedPtr client_;
    std::string bt_node_name_;

    float criteria_min_{0.00};
    float criteria_max_{14.00};
    bool configured_{false};

    void configureOnce() { // Read ports once (first tick) and cache
        if (configured_) return;
        (void)getInput<float>(std::string("criteria_min"), criteria_min_);
        (void)getInput<float>(std::string("criteria_max"), criteria_max_);
        configured_ = true;
    }
    
};

#endif // CHECK_DPH_DV_CONDITION_H