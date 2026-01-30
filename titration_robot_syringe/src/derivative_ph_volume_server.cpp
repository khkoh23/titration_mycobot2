#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <titration_robot_interfaces/srv/derivative_ph_volume.hpp>

#include <deque>
#include <mutex>
#include <chrono>

class DerivativePhVolumeServiceServer : public rclcpp::Node {
public:
    using DerivativePhVolume = titration_robot_interfaces::srv::DerivativePhVolume;
    DerivativePhVolumeServiceServer() : Node("derivative_ph_volume_server") {
        using namespace std::placeholders;
        ph_subscription_ = this->create_subscription<std_msgs::msg::Float32>("ph", 10, std::bind(&DerivativePhVolumeServiceServer::ph_callback, this, _1));
        titration_vol_subscription_ = this->create_subscription<std_msgs::msg::UInt32>("titration_vol", 10, std::bind(&DerivativePhVolumeServiceServer::titration_vol_callback, this, _1));
        service_ = this->create_service<DerivativePhVolume>("derivative_ph_volume", std::bind(&DerivativePhVolumeServiceServer::handle_service, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "DerivativePhVolume service server started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ph_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr titration_vol_subscription_;
    rclcpp::Service<titration_robot_interfaces::srv::DerivativePhVolume>::SharedPtr service_; 
    struct VolPhRepaired {
        float volume{0.0f}; // cumulative titration volume (0.1 µL units)
        float ph; // pH value sampled at that time
        bool repaired{false}; // whether pH data is overwritten with a stable reading
        std::chrono::steady_clock::time_point t{}; // timestamp for observability or potential future guards
    };
    std::deque<VolPhRepaired> ph_vol_data_;
    float latest_ph_ = 0.0f;
    std::mutex data_mutex_;

    void ph_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_ph_ = msg->data;
    }

    void titration_vol_callback(const std_msgs::msg::UInt32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto now = std::chrono::steady_clock::now();
        // Store (volume, pH_at_dispense) pairs. Limit history to a few points for derivative calculation.
        if (!ph_vol_data_.empty() && ph_vol_data_.back().volume == static_cast<float>(msg->data)) {
            // Defensive path: in case the same titration_vol is republished, refresh the pH for that volume.
            ph_vol_data_.back().ph = latest_ph_;
            ph_vol_data_.back().t = now;
            // Do not set repaired here; this is the "at-publish" refresh, not the post-stability repair.
        }
        else {
            ph_vol_data_.push_back(VolPhRepaired{static_cast<float>(msg->data), latest_ph_, false, now});
            if (ph_vol_data_.size() > 5) { // Keep a small history for derivative
                ph_vol_data_.pop_front();
            }
        }
    }

    void handle_service(const std::shared_ptr<DerivativePhVolume::Request>, std::shared_ptr<DerivativePhVolume::Response> response) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // --- Repair-on-request: assume BT ensured stability before this call ---
        // Overwrite the last sample's pH with the stabilized latest_ph_ exactly once.
        if (!ph_vol_data_.empty()) {
            auto &last = ph_vol_data_.back();
            if (!last.repaired) {
                last.ph = latest_ph_;
                last.repaired = true;
                // TODO: Add time 
                // last.t = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(), "Repaired last pair to stable pH: V=%.1f (0.1 uL), pH=%.3f", last.volume, last.ph);
            }
        }
        if (ph_vol_data_.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough data points to calculate derivative.");
            response->dph_dv = 0.0; // TODO: Add a guard
            return;
        }
        const auto &s2 = ph_vol_data_[ph_vol_data_.size() - 1];
        const auto &s1 = ph_vol_data_[ph_vol_data_.size() - 2];
        float dV_raw = s2.volume - s1.volume; // difference in 0.1 µL units
        float dPh = s2.ph - s1.ph;
        if (dV_raw == 0.0f) {
            RCLCPP_WARN(this->get_logger(), "Change in volume is zero, cannot calculate derivative.");
            response->dph_dv = 0.0; // TODO: Add a guard
            return;
        }
        response->dph_dv = dPh / dV_raw;
        RCLCPP_INFO(this->get_logger(), "Calculated dPh/dV (per 0.1 uL): %f", response->dph_dv);
  }

}; // class DerivativePhVolumeServiceServer

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DerivativePhVolumeServiceServer>());
    rclcpp::shutdown();
    return 0;
}