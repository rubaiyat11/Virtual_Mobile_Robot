#include "rclcpp/rclcpp.hpp"

class sensor_node : public rclcpp::Node{
public:
    sensor_node() : Node("sensor_node"){
        RCLCPP_INFO(this->get_logger(), "Sensor node working");
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}