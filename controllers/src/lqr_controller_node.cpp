#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/desired_state.hpp"
#include "mission_interface/msg/adjusted_state.hpp"

class lqr_controller_node : public rclcpp::Node{
public:
    lqr_controller_node() : Node("lqr_controller_node"){
        desired_state_sub = this->create_subscription<mission_interface::msg::DesiredState>(
            "/desired_state", 20, std::bind(&lqr_controller_node::desired_state_callback, this, std::placeholders::_1)
        );

        adjusted_state_pub = this->create_publisher<mission_interface::msg::AdjustedState>(
            "/adjusted_state", 20
        );
    }

private:

    void desired_state_callback(const mission_interface::msg::DesiredState::SharedPtr msg){

        x(0) = msg->desired_position[0];
        x(1) = msg->desired_position[1];
        x(2) = msg->desired_position[2];

        x(3) = msg->desired_velocity[0];
        x(4) = msg->desired_velocity[1];
        x(5) = msg->desired_velocity[2];
    }

    rclcpp::Subscription<mission_interface::msg::DesiredState>::SharedPtr desired_state_sub;
    rclcpp::Publisher<mission_interface::msg::AdjustedState>::SharedPtr adjusted_state_pub;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lqr_controller_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}