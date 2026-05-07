#include "rclcpp/rclcpp.hpp"

class plant_node : public rclcpp::Node{
public:
    plant_node() : Node("plant_nodes"){
        RCLCPP_INFO(this->get_logger(), "Plant Node working");
        
    }
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
// Publishes that to plant state topic for other nodes to use