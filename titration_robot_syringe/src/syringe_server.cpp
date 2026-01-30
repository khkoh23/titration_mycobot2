#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <titration_robot_interfaces/srv/syringe.hpp>

#include <memory>

class SyringeServiceServer : public rclcpp::Node {
public:

    using Syringe = titration_robot_interfaces::srv::Syringe;
    explicit SyringeServiceServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("syringe_server", options) {
        using namespace std::placeholders;
        syringe_cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>("syringe_cmd", rclcpp::QoS(10).reliable()); 
        syringe_set_vol_pub_ = this->create_publisher<std_msgs::msg::UInt16>("syringe_set_vol", rclcpp::QoS(1).reliable().transient_local());
        syringe_step_vol_pub_ = this->create_publisher<std_msgs::msg::UInt16>("syringe_step_vol", rclcpp::QoS(1).reliable().transient_local());
        titration_vol_pub_ = this->create_publisher<std_msgs::msg::UInt32>("titration_vol", rclcpp::QoS(1).reliable().transient_local());
        this->service_ = this->create_service<Syringe>("syringe", std::bind(&SyringeServiceServer::handle_request, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Syringe service server started.");
    }

private:
    rclcpp::Service<Syringe>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr syringe_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr syringe_set_vol_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr syringe_step_vol_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr titration_vol_pub_;
    const int32_t capacity_volume_ = 50000; // maximum 5000.0 uL syringe volume
    int32_t holding_volume_ = 50000; // curent liquid held by the syringe
    uint16_t aspire_volume_ = 20000; // 2000.0 uL
    uint16_t dispense_volume_ = 200; // 20.0 uL
    uint32_t titration_volume = 0;

    inline void help_publish_int8(rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub, int8_t value) {
        std_msgs::msg::Int8 msg;
        msg.data = value;
        pub->publish(msg);
    }

    inline void help_publish_uint16(rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub, uint16_t value) {
        std_msgs::msg::UInt16 msg;
        msg.data = value;
        pub->publish(msg);
    }

    inline void help_publish_uint32(rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub, uint32_t value) {
        std_msgs::msg::UInt32 msg;
        msg.data = value;
        pub->publish(msg);
    }

    void handle_request(const std::shared_ptr<Syringe::Request> request, std::shared_ptr<Syringe::Response> response) {
        switch (request->command) {
            case 7:
                if (holding_volume_ + aspire_volume_ <= capacity_volume_) {
                    holding_volume_ += aspire_volume_;
                    help_publish_int8(syringe_cmd_pub_, 7);
                    RCLCPP_INFO(this->get_logger(), "Aspired. Holding volume: %d/%d.", holding_volume_, capacity_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Aspire failed: Holding volume after Aspire %d is greater than max volume %d.", aspire_volume_ + holding_volume_, capacity_volume_);
                break;
            case 71:
                if (request->aspire_volume >= 50 && request->aspire_volume <= 50000) { // accept range 5 uL to 5000 uL
                    aspire_volume_ = request->aspire_volume;
                    help_publish_uint16(syringe_set_vol_pub_, aspire_volume_);
                    RCLCPP_INFO(this->get_logger(), "Aspire volume is set to: %d.", aspire_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Aspire volume is outside range 50 to 50000.");
                break;
            case 8: // Clear syringe
                help_publish_uint16(syringe_set_vol_pub_, holding_volume_);
                holding_volume_ = 0;
                help_publish_int8(syringe_cmd_pub_, 8);
                RCLCPP_WARN(this->get_logger(), "Syringe cleared.");
                break;
            case 9:
                if (holding_volume_ >= dispense_volume_) {
                    holding_volume_ -= dispense_volume_;
                    help_publish_int8(syringe_cmd_pub_, 9);
                    titration_volume += dispense_volume_;
                    help_publish_uint32(titration_vol_pub_, titration_volume);
                    RCLCPP_INFO(this->get_logger(), "Dispensed. Holding volume: %d/%d.", holding_volume_, capacity_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Dispense failed: Holding volume %d is lesser than dispense step volume %d.", holding_volume_, dispense_volume_);
                break;
            case 91:
                if (request->dispense_volume >= 50 && request->dispense_volume <= 50000) { // accept range 5 uL to 5000 uL
                    dispense_volume_ = request->dispense_volume;
                    help_publish_uint16(syringe_step_vol_pub_, dispense_volume_);
                    RCLCPP_INFO(this->get_logger(), "Dispense volume is set to: %d.", dispense_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Dispense volume is outside range 50 to 50000.");
                break;
            case 123:
                RCLCPP_INFO(this->get_logger(), "Response to holding volume enquiry: %d/%d.", holding_volume_, capacity_volume_);
                break;
            case 124:
                if (dispense_volume_ > holding_volume_) {
                    RCLCPP_WARN(this->get_logger(), "Yes to replenish ...");
                    response->to_replenish = true;
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "No need to replenish.");
                    response->to_replenish = false;
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command code: %d", static_cast<int>(request->command));
                break;
        }
        response->holding_volume = holding_volume_;
    }

}; // class SyringeServiceServer

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto service_server = std::make_shared<SyringeServiceServer>(node_options);
    rclcpp::spin(service_server);
    rclcpp::shutdown();
    return 0;
}