#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/robot_state.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include <Eigen/Dense>
#include <random>

class sensor_node : public rclcpp::Node{
public:
    sensor_node() : Node("sensor_node"){

        plant_state_sub = this->create_subscription<mission_interface::msg::RobotState>(
            "/plant_state", 10, std::bind(&sensor_node::plant_state_reciever, this, std::placeholders::_1)
        );

        measured_state_pub = this->create_publisher<mission_interface::msg::SensorState>(
            "/measured_state", 10
        );
    
        measured_state_updater = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&sensor_node::timer_callback, this)
        );
    }

private:

    void plant_state_reciever(const mission_interface::msg::RobotState::SharedPtr msg){

        actual_position.x() = msg->position[0];
        actual_position.y() = msg->position[1];
        actual_position.z() = msg->position[2];

        actual_velocity.x() = msg->velocity[0];
        actual_velocity.y() = msg->velocity[1];
        actual_velocity.z() = msg->velocity[2];

        actual_acceleration.x() = msg->acceleration[0];
        actual_acceleration.y() = msg->acceleration[1];
        actual_acceleration.z() = msg->acceleration[2];

    }

    void timer_callback(){
        mission_interface::msg::SensorState msg;

        msg.stamp = this->get_clock()->now();

        msg.measured_position[0] = actual_position.x() + noise(engine);
        msg.measured_position[1] = actual_position.y() + noise(engine);
        msg.measured_position[2] = actual_position.z() + noise(engine);

        msg.measured_velocity[0] = actual_velocity.x() + noise(engine);
        msg.measured_velocity[1] = actual_velocity.y() + noise(engine);
        msg.measured_velocity[2] = actual_velocity.z() + noise(engine);

        msg.measured_acceleration[0] = actual_acceleration.x() + noise(engine);
        msg.measured_acceleration[1] = actual_acceleration.y() + noise(engine);
        msg.measured_acceleration[2] = actual_acceleration.z() + noise(engine);

        measured_state_pub->publish(msg);
        
    }

    Eigen::Vector3d actual_position{0.0, 0.0, 0.0};
    Eigen::Vector3d actual_velocity{0.0, 0.0, 0.0};
    Eigen::Vector3d actual_acceleration{0.0, 0.0, 0.0};
    
    std::random_device rd;
    std::default_random_engine engine{rd()};
    std::normal_distribution<double> noise{0.0, 0.05};

    rclcpp::Subscription<mission_interface::msg::RobotState>::SharedPtr plant_state_sub;
    rclcpp::Publisher<mission_interface::msg::SensorState>::SharedPtr measured_state_pub;
    rclcpp::TimerBase::SharedPtr measured_state_updater;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}