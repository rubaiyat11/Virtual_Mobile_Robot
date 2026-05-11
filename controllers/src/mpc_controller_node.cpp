#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include "mission_interface/msg/target_state.hpp"
#include "mission_interface/msg/desired_state.hpp"

class mpc_ctrlr_node : public rclcpp::Node{
public:
    mpc_ctrlr_node() : Node("mpc_ctrlr_node"){
        measured_plant_state_sub = this->create_subscription<mission_interface::msg::SensorState>(
            "/measured_state", 20, std::bind(&mpc_ctrlr_node::measured_state_callback, this, std::placeholders::_1)
        );

        target_state_sub = this->create_subscription<mission_interface::msg::TargetState>(
            "/target_state", 20, std::bind(&mpc_ctrlr_node::target_state_callback, this, std::placeholders::_1)
        );

        mpc_output_pub = this->create_publisher<mission_interface::msg::DesiredState>(
            "/desired_state", 20
        );

        mpc_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&mpc_ctrlr_node::timer_callback, this)
        );
    }

private:

    void measured_state_callback(const mission_interface::msg::SensorState::SharedPtr msg){

    }

    void target_state_callbaack(const mission_interface::msg::TargetState::SharedPtr msg){

    }

    void timer_callback(){

    }

    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_plant_state_sub;
    rclcpp::Subscription<mission_interface::msg::TargetState>::SharedPtr target_state_sub;
    rclcpp::Publisher<mission_interface::msg::DesiredState>::SharedPtr mpc_output_pub;  
    const mission_interface::msg::SensorState::SharedPtr msg;
};



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpc_ctrlr_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}