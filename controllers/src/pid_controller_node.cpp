#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/adjusted_state.hpp"
#include "mission_interface/msg/control_effort.hpp"

class pid_controller_node : public rclcpp::Node{
public:
    mpc_controller_node() : Node("pid_controller_node"){
        adjusted_state_sub = this->create_subscription<mission_interface::msg::AdjustedState>(
            "/adjusted_state", 20, std::bind(&pid_controller_node::adjusted_state_callback, this, std::placeholders::_1)
        );

        control_effort_pub = this->create_publisher<mission_interface::msg::ControlEffort>(
            "/control_effort", 20
        );
    }

private:
    void adjusted_state_callback(const mission_interface::msg::AdjustedState::SharedPtr msg){

        x(0) = msg->adjusted_position[0];
        x(1) = msg->adjusted_position[1];
        x(2) = msg->adjusted_position[2];

        x(3) = msg->adjusted_velocity[0];
        x(4) = msg->adjusted_velocity[1];
        x(5) = msg->adjusted_velocity[2];

    }

    rclcpp::Subscription<mission_interface::msg::AdjustedState>::SharedPtr adjusted_state_sub;
    rclcpp::Publisher<mission_interface::msg::control_effort>::SharedPtr control_effort_pub;
}

int main(int argc, char **argv){
    rclcp::init(argc, argv);
    auto node = std::make_shared<pid_controller_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}