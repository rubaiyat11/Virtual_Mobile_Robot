#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/adjusted_state.hpp"
#include "mission_interface/msg/robot_state.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include <Eigen/Dense>
#include <algorithm>

class pid_controller_node : public rclcpp::Node {
public:
    pid_controller_node() : Node("pid_controller_node") {
        
        adjusted_state_sub = this->create_subscription<mission_interface::msg::AdjustedState>(
            "/adjusted_state", 20, std::bind(&pid_controller_node::adjusted_state_callback, this, std::placeholders::_1)
        );

        
        measured_state_sub = this->create_subscription<mission_interface::msg::SensorState>(
            "/measured_state", 20, std::bind(&pid_controller_node::measured_state_callback, this, std::placeholders::_1)
        );

        
        control_effort_pub = this->create_publisher<mission_interface::msg::RobotState>(
            "/controller_output", 20
        );

        double loop_rate_ms = 10.0;
        dt = loop_rate_ms / 1000.0;

        pid_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(loop_rate_ms)),
            std::bind(&pid_controller_node::timer_callback, this)
        );

        a_target.setZero();
        a_actual.setZero();
        integral_error.setZero();
        prev_error.setZero();
        derivative_error.setZero();

        Kp << 1.2, 1.2, 1.2;
        Kd << 0.4, 0.4, 0.4;
        Ki << 0.05, 0.05, 0.05;
        
        new_data_available = false;
    }

private:
    void adjusted_state_callback(const mission_interface::msg::AdjustedState::SharedPtr msg) {
        
        a_target(0) = msg->adjusted_acceleration[0];
        a_target(1) = msg->adjusted_acceleration[1];
        a_target(2) = msg->adjusted_acceleration[2];
    }

    void measured_state_callback(const mission_interface::msg::SensorState::SharedPtr msg) {
        a_actual(0) = msg->measured_position[0];
        a_actual(1) = msg->measured_position[1];
        a_actual(2) = msg->measured_position[2];
        new_data_available = true;
    }

    void timer_callback() {
    
        Eigen::Vector3d error = a_target - a_actual;

        if (error.norm() < 2.0) { 
            integral_error += error * dt;
            double max_i = 5.0;
            integral_error = integral_error.cwiseMax(-max_i).cwiseMin(max_i);
        } else {
            integral_error.setZero();
        }
    
        if (new_data_available) {
            derivative_error = (error - prev_error) / dt;
            prev_error = error;
            new_data_available = false;
        }

        Eigen::Vector3d u_pid = Kp.cwiseProduct(error) + Ki.cwiseProduct(integral_error) + Kd.cwiseProduct(derivative_error);
        Eigen::Vector3d force_command = u_pid * mass;

        mission_interface::msg::RobotState out_msg;
        out_msg.force[0] = force_command(0);
        out_msg.force[1] = force_command(1);
        out_msg.force[2] = force_command(2);

        control_effort_pub->publish(out_msg);
    }

    rclcpp::Subscription<mission_interface::msg::AdjustedState>::SharedPtr adjusted_state_sub;
    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_state_sub;
    rclcpp::Publisher<mission_interface::msg::RobotState>::SharedPtr control_effort_pub;
    rclcpp::TimerBase::SharedPtr pid_pub_timer;

    Eigen::Vector3d a_target;
    Eigen::Vector3d a_actual;
    Eigen::Vector3d integral_error;
    Eigen::Vector3d prev_error;
    Eigen::Vector3d derivative_error;
    Eigen::Vector3d Kp, Ki, Kd;

    double dt;
    double mass = 1.0;
    bool new_data_available;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pid_controller_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}