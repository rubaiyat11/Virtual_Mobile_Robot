#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include "mission_interface/msg/target_state.hpp"
#include "mission_interface/msg/desired_state.hpp"

class mpc_ctrlr_node : public rclcpp::Node{
public:
    mpc_ctrlr_node() : Node("mpc_ctrlr_node"){
        measured_plant_state_sub = this->create_subscription<mission_interface::msg::SensorState>(
            "/measured_state", 20, std::bind(&mpc_ctrlr_node::measured_state_callback, this, std::placeholder::_1)
        );

        target_state_sub = this->create_subscription<mission_interface::msg::TargetState>(
            "/target_state", 20, std::bind(&mpc_ctrlr_node::target_state_callback, this, std::placeholders::_1)
        );

        mpc_output_pub = this->create_publisher<mission_interface::msg::DesiredState>(
            "/desired_state", 20
        );
    }

private:
    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_plant_state_sub;
    rclcpp::Subscription<mission_interface::msg::TargetState>::SharedPtr target_state_sub;
    rclcpp::Publisher<mission_interface::msg::DesiredState>::SharedPtr mpc_output_pub;  
};



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<plant_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}