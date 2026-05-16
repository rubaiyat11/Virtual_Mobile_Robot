#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include "mission_interface/msg/target_state.hpp"
#include "mission_interface/msg/desired_state.hpp"
#include <Eigen/Dense>

class mpc_controller_node : public rclcpp::Node{
public:
    mpc_controller_node() : Node("mpc_controller_node"){
        measured_plant_state_sub = this->create_subscription<mission_interface::msg::SensorState>(
            "/measured_state", 20, std::bind(&mpc_controller_node::measured_state_callback, this, std::placeholders::_1)
        );              //Taking data from sensor node

        target_state_sub = this->create_subscription<mission_interface::msg::TargetState>(
            "/target_state", 20, std::bind(&mpc_controller_node::target_state_callback, this, std::placeholders::_1)
        );              //Taking data from user command

        mpc_output_pub = this->create_publisher<mission_interface::msg::DesiredState>(
            "/desired_state", 20
        );              //Sending data for optimal state 

        mpc_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&mpc_controller_node::timer_callback, this)
        );

        A <<
        1,0,0,dt,0,0,
        0,1,0,0,dt,0,
        0,0,1,0,0,dt,

        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;

        B <<
        0,0,0,
        0,0,0,
        0,0,0,

        dt,0,0,
        0,dt,0,
        0,0,dt;

        Q <<
        20,0,0,0,0,0,
        0,20,0,0,0,0,
        0,0,20,0,0,0,

        0,0,0,5,0,0,
        0,0,0,0,5,0,
        0,0,0,0,0,5;

        R <<
        0.2,0,0,
        0,0.2,0,
        0,0,0.2;
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
        error = x_target - x;

        u = Eigen::Vector3d(
        error(0) * 2.0,
        error(1) * 2.0,
        error(2) * 2.0
    );

    desired_position = x.head<3>() + x.tail<3>() * dt;
    desired_velocity = x.tail<3>() + u * dt;

    }

    void timer_callback(){

        compute_mpc();

        mission_interface::msg::DesiredState msg;

        msg.desired_position[0] = desired_position.x();
        msg.desired_position[1] = desired_position.y();
        msg.desired_position[2] = desired_position.z();

        msg.desired_velocity[0] = desired_velocity.x();
        msg.desired_velocity[1] = desired_velocity.y();
        msg.desired_velocity[2] = desired_velocity.z();

        msg.desired_acceleration[0] = u(0);
        msg.desired_acceleration[1] = u(1);
        msg.desired_acceleration[2] = u(2);

        mpc_output_pub->publish(msg);
    }


    double dt = 0.01;

    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> x;
    Eigen::Matrix<double, 6, 1> x_target;
    Eigen::Vector3d u;

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;

    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;

    Eigen::Vector3d desired_position;
    Eigen::Vector3d desired_velocity;
    

    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_plant_state_sub;
    rclcpp::Subscription<mission_interface::msg::TargetState>::SharedPtr target_state_sub;
    rclcpp::Publisher<mission_interface::msg::DesiredState>::SharedPtr mpc_output_pub;  
    rclcpp::TimerBase::SharedPtr mpc_pub_timer;
};



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpc_controller_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}