#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/desired_state.hpp"
#include "mission_interface/msg/adjusted_state.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include <Eigen/Dense>

class lqr_controller_node : public rclcpp::Node{
public:
    lqr_controller_node() : Node("lqr_controller_node"){
        desired_state_sub = this->create_subscription<mission_interface::msg::DesiredState>(
            "/desired_state", 20, std::bind(&lqr_controller_node::desired_state_callback, this, std::placeholders::_1)
        );

        measured_state_sub = this->create_subscription<mission_interface::msg::SensorState>(
            "/measured_state", 20, std::bind(&lqr_controller_node::measured_state_callback, this, std::placeholders::_1)
        );

        adjusted_state_pub = this->create_publisher<mission_interface::msg::AdjustedState>(
            "/adjusted_state", 20
        );

        lqr_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&lqr_controller_node::timer_callback, this)
        );

        x_desired.setZero();
        x_actual.setZero();
        u_mpc_ff.setZero();
        
        K <<
        5.0, 0.0, 0.0, 3.0, 0.0, 0.0,
        0.0, 5.0, 0.0, 0.0, 3.0, 0.0,
        0.0, 0.0, 5.0, 0.0, 0.0, 3.0;
    }

private:

    void desired_state_callback(const mission_interface::msg::DesiredState::SharedPtr msg){

        x_desired(0) = msg->desired_position[0];
        x_desired(1) = msg->desired_position[1];
        x_desired(2) = msg->desired_position[2];

        x_desired(3) = msg->desired_velocity[0];
        x_desired(4) = msg->desired_velocity[1];
        x_desired(5) = msg->desired_velocity[2];

        u_mpc_ff(0) = msg->desired_acceleration[0];
        u_mpc_ff(1) = msg->desired_acceleration[1];
        u_mpc_ff(2) = msg->desired_acceleration[2];
    }

    void measured_state_callback(const mission_interface::msg::SensorState::SharedPtr msg){
        x_actual(0) = msg->measured_position[0];
        x_actual(1) = msg->measured_position[1];
        x_actual(2) = msg->measured_position[2];

        x_actual(3) = msg->measured_velocity[0];
        x_actual(4) = msg->measured_velocity[1];
        x_actual(5) = msg->measured_velocity[2];
    }

    void timer_callback(){
        x_desired.setZero();
        x_desired(0) = 1.0;

        Eigen::Matrix<double, 6, 1> x_error = x_desired - x_actual;

        Eigen::Vector3d u_lqr = K * x_error;

        Eigen::Vector3d adjusted_acceleration = u_lqr;  // + u_mpc_ff ;

        mission_interface::msg::AdjustedState out_msg;

        out_msg.adjusted_acceleration[0] = adjusted_acceleration(0);
        out_msg.adjusted_acceleration[1] = adjusted_acceleration(1);
        out_msg.adjusted_acceleration[2] = adjusted_acceleration(2);

        adjusted_state_pub->publish(out_msg);
    }

    rclcpp::Subscription<mission_interface::msg::DesiredState>::SharedPtr desired_state_sub;
    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_state_sub;
    rclcpp::Publisher<mission_interface::msg::AdjustedState>::SharedPtr adjusted_state_pub;
    rclcpp::TimerBase::SharedPtr lqr_pub_timer;

    Eigen::Matrix<double, 6, 1> x_desired;
    Eigen::Matrix<double, 6, 1> x_actual;
    Eigen::Vector3d u_mpc_ff;

    Eigen::Matrix<double, 3, 6> K;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lqr_controller_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}