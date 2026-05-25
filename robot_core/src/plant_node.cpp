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
        position.x() = msg->position[0];
        position.y() = msg->position[1];
        position.z() = msg->position[2];
    }

    void timer_callback(){
        mission_interface::msg::RobotState msg;

        //velocity = velocity + acceleration * dt;      //v: velocity, dt: time frame
        position = position; //+ velocity * dt;      //x: updated_position

        msg.position[0] = position.x();
        msg.position[1] = position.y();
        msg.position[2] = position.z();

        //msg.velocity[0] = velocity.x();
        //msg.velocity[1] = velocity.y();
        //msg.velocity[2] = velocity.z();


        plant_state_pub->publish(msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                    "Pos: x=%.2f, y=%.2f, z=%.2f", position.x(), position.y(), position.z());
    }

    Eigen::Vector3d position{0.0, 0.0, 0.0};
    Eigen::Vector3d velocity{0.0, 0.0, 0.0};
    Eigen::Vector3d acceleration{0.0, 0.0, 0.0};
    Eigen::Vector3d force{0.0, 0.0, 0.0};

    double m = 1.0;
    double dt = 0.01;

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
