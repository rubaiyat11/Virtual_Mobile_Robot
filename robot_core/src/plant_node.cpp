#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mission_interface/msg/robot_state.hpp"
#include <Eigen/Dense>

class plant_node : public rclcpp::Node{
public:
    plant_node() : Node("plant_node"){
        controller_sub = this->create_subscription<mission_interface::msg::RobotState>(
            "/controller_output", 20, std::bind(&plant_node::control_input_callback, this, std::placeholders::_1)
        );

        plant_state_pub = this->create_publisher<mission_interface::msg::RobotState>(
            "/plant_state", 20
        );

        state_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&plant_node::timer_callback, this)
        );

    }
private:
    void control_input_callback(const mission_interface::msg::RobotState::SharedPtr msg){
        force.x() = msg->force[0];
        force.y() = msg->force[1];
        force.z() = msg->force[2];   //F: latest_force
    }

    void timer_callback(){
        mission_interface::msg::RobotState msg;

        acceleration = force/m;             //m: mass of system, a: updated_acceleration
        velocity = velocity + acceleration * dt;      //v: velocity, dt: time frame
        position = position + velocity * dt;      //x: updated_position

        msg.position[0] = position.x();
        msg.position[1] = position.y();
        msg.position[2] = position.z();

        msg.velocity[0] = velocity.x();
        msg.velocity[1] = velocity.y();
        msg.velocity[2] = velocity.z();

        msg.acceleration[0] = acceleration.x();
        msg.acceleration[1] = acceleration.y();
        msg.acceleration[2] = acceleration.z();

        plant_state_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Pos: x=%f, y=%f, z=%f", position.x(), position.y(), position.z());
    }

    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    Eigen::Vector3d acceleration{0.0, 0.0, 0.0};
    Eigen::Vector3d force{0.0, 0.0, 0.0};

    double m = 1.0;
    double dt = 0.1;

    rclcpp::Subscription<mission_interface::msg::RobotState>::SharedPtr controller_sub;
    rclcpp::Publisher<mission_interface::msg::RobotState>::SharedPtr plant_state_pub;
    rclcpp::TimerBase::SharedPtr state_pub_timer;       
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<plant_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Takes input from controllers
// Controllers gives force
// Plant turns them into velocity and position (real body state)
// Publishes that to plant state topic for other nodes to uses