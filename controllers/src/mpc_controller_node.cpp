#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include "mission_interface/msg/target_state.hpp"
#include "mission_interface/msg/desired_state.hpp"
#include <Eigen/Dense>

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

        x(0) = msg->measured_position[0];
        x(1) = msg->measured_position[1];
        x(2) = msg->measured_position[2];

        x(3) = msg->measured_velocity[0];
        x(4) = msg->measured_velocity[1];
        x(5) = msg->measured_velocity[2];

    }

    void target_state_callback(const mission_interface::msg::TargetState::SharedPtr msg){
        
        x_target(0) = msg->target_position[0];
        x_target(1) = msg->target_position[1];
        x_target(2) = msg->target_position[2];

        x_target(3) = msg->target_velocity[0];
        x_target(4) = msg->target_velocity[1];
        x_target(5) = msg->target_velocity[2];

    }

    void compute_mpc(){

    }

    void timer_callback(){

    }


    double dt = 0.01;

    Eigen::Matrix<double, 6, 1> x;
    Eigen::Matrix<double, 6, 1> x_target;
    Eigen::Vector3d u;

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;

    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;

    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_plant_state_sub;
    rclcpp::Subscription<mission_interface::msg::TargetState>::SharedPtr target_state_sub;
    rclcpp::Publisher<mission_interface::msg::DesiredState>::SharedPtr mpc_output_pub;  
    rclcpp::TimerBase::SharedPtr mpc_pub_timer;
};



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpc_ctrlr_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}