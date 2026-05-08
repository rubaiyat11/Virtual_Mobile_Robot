#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class plant_node : public rclcpp::Node{
public:
    plant_node() : Node("plant_node"){
        controller_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/controller_output", 20, std::bind(&plant_node::control_input_callback, this, std::placeholders::_1)
        );

        plant_state_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            "/plant_state", 20
        );

        state_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&plant_node::timer_callback, this)
        );

    }
private:
    void control_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        F = msg->linear.x;   //F: latest_force
    }

    void timer_callback(){
        geometry_msgs::msg::Twist msg;

        a = F/m;             //m: mass of system, a: updated_acceleration
        v = v + a * dt;      //v: velocity, dt: time frame
        x = x + v * dt;      //x: updated_position

        msg.linear.x = x;
        msg.linear.y = v;
        msg.linear.z = a;

        plant_state_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "x: %f", x);
    }

    double F = 0.0;
    double a = 0.0;
    double v = 0.0;
    double x = 0.0;

    double m = 1.0;
    double dt = 0.01;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr plant_state_pub;
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