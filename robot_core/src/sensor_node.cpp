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

        rclcpp::QoS qos(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        measured_state_pub = this->create_publisher<mission_interface::msg::SensorState>(
            "/measured_state", qos
        );

        RCLCPP_INFO(this->get_logger(), "Sensor node working");
    
        measured_state_updater = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&sensor_node::timer_callback, this)
        );
    }

private:

    void plant_state_reciever(const mission_interface::msg::RobotState::SharedPtr msg){

        measured_position.x() = msg->position[0];
        measured_position.y() = msg->position[1];
        measured_position.z() = msg->position[2];

        measured_velocity.x() = msg->velocity[0];
        measured_velocity.y() = msg->velocity[1];
        measured_velocity.z() = msg->velocity[2];

    }

    void timer_callback(){
        mission_interface::msg::SensorState msg;

        double noise_sample = noise(engine);

        measured_position.x() += noise_sample;
        measured_position.y() += noise_sample;
        measured_position.z() += noise_sample;

        measured_velocity.x() += noise_sample;
        measured_velocity.y() += noise_sample;
        measured_velocity.z() += noise_sample;

        msg.measured_position[0] = measured_position.x();
        msg.measured_position[1] = measured_position.y();
        msg.measured_position[2] = measured_position.z();

        msg.measured_velocity[0] = measured_velocity.x();
        msg.measured_velocity[1] = measured_velocity.y();
        msg.measured_velocity[2] = measured_velocity.z();

        measured_state_pub->publish(msg);
        
        msg.stamp = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Pos: x=%f, y=%f, z=%f", measured_position.x(), measured_position.y(), measured_position.z());
    }

    Eigen::Vector3d measured_position{0.0, 0.0, 0.0};
    Eigen::Vector3d measured_velocity{0.0, 0.0, 0.0};
    
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