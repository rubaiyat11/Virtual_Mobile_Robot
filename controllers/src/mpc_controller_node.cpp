#include "rclcpp/rclcpp.hpp"
#include "mission_interface/msg/sensor_state.hpp"
#include "mission_interface/msg/target_state.hpp"
#include "mission_interface/msg/desired_state.hpp"
#include "mission_interface/msg/reference_state.hpp"
#include <Eigen/Dense>

class mpc_controller_node : public rclcpp::Node{
public:
    mpc_controller_node() : Node("mpc_controller_node"){
        measured_plant_state_sub = this->create_subscription<mission_interface::msg::SensorState>(
            "/measured_state", 20, std::bind(&mpc_controller_node::measured_state_callback, this, std::placeholders::_1)
        );

        planner_state_sub = this->create_subscription<mission_interface::msg::ReferenceState>(
            "/reference_state", 20, std::bind(&mpc_controller_node::reference_state_callback, this, std::placeholders::_1)
        );

        mpc_output_pub = this->create_publisher<mission_interface::msg::DesiredState>(
            "/desired_state", 20
        );   

        mpc_pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&mpc_controller_node::timer_callback, this)
        );

        x.setZero();
        x_target.setZero();
        x_pred.setZero();
        u.setZero();

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

        init_horizon_matrices();
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

    void reference_state_callback(const mission_interface::msg::ReferenceState::SharedPtr msg){
        
        x_target(0) = msg->position_trajectory[0];
        x_target(1) = msg->position_trajectory[1];
        x_target(2) = msg->position_trajectory[2];

        x_target(3) = msg->velocity_trajectory[0];
        x_target(4) = msg->velocity_trajectory[1];
        x_target(5) = msg->velocity_trajectory[2];

    }

    void init_horizon_matrices(){
        P_x.resize(6 * H, 6);
        H_x.resize(6 * H, 3 * H);
        H_x.setZero();

        Q_bar.resize(6 * H, 6 * H); Q_bar.setZero();
        R_bar.resize(3 * H, 3 * H); R_bar.setZero();

        g.resize(3 * H, 1);

        for(int i = 0; i < H; i++){
            Q_bar.block<6, 6>(i * 6, i * 6) = Q;
            R_bar.block<3, 3>(i * 3, i * 3) = R;
        }

        Eigen::Matrix<double, 6, 6> A_power = A;

        for(int i =0; i < H; ++i){
            P_x.block<6, 6>(i * 6, 0) = A_power;

            for(int j =0; j <= i; ++j){
                Eigen::Matrix<double, 6, 6> A_diff = Eigen::Matrix<double, 6, 6>::Identity();
                
                for(int k=0; k < (i-j); ++k){
                    A_diff = A_diff * A;
                }
                H_x.block<6, 3>(i * 6, j *3) = A_diff * B;
            }
            A_power = A_power * A;
        }

        H_qp = 2.0 * (H_x.transpose() * Q_bar * H_x + R_bar);

    }

    void update_qp_gradient(){
        Eigen::VectorXd X_target(6 * H);

        for(int i = 0; i < H; ++i){
            X_target.segment<6>(i * 6) = x_target;
        }

        g = 2.0 * H_x.transpose() * Q_bar * (P_x * x - X_target);
    }

    void compute_mpc(){
        update_qp_gradient();
        
        Eigen::VectorXd U_optimal = -H_qp.ldlt().solve(g);

        u(0) = U_optimal(0);
        u(1) = U_optimal(1);
        u(2) = U_optimal(2);

        x_pred = A * x + B * u;
    }

    void timer_callback(){
        cost = 0.0;
        
        compute_mpc();


        Eigen::VectorXd U_horizon = Eigen::VectorXd::Zero(3 * H);

        U_horizon.segment<3>(0) = u;

        Eigen::VectorXd X_predicted_profile = P_x * x + H_x * U_horizon;

        Eigen::VectorXd X_target_profile = Eigen::VectorXd::Zero(6 * H);

        for(int i = 0; i < H; ++i){
            X_target_profile.segment<6>(i * 6) = x_target;
        }

        Eigen::VectorXd tracking_error_profile = X_target_profile - X_predicted_profile;

        cost += (tracking_error_profile.transpose() * Q_bar * tracking_error_profile).value();
        cost += (U_horizon.transpose() * R_bar * U_horizon).value();


        mission_interface::msg::DesiredState msg;

        msg.desired_position = {x_pred(0), x_pred(1), x_pred(2)};
        msg.desired_velocity = {x_pred(3), x_pred(4), x_pred(5)};

        msg.desired_acceleration = {u(0), u(1), u(2)};

        mpc_output_pub->publish(msg);
    }


    double dt = 0.01;
    int H = 10;
    double cost = 0.0;

    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> x;
    Eigen::Matrix<double, 6, 1> x_target;
    Eigen::Matrix<double, 6, 1> x_pred;
    Eigen::Vector3d u;

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;

    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;

    Eigen::MatrixXd P_x;
    Eigen::MatrixXd H_x;
    Eigen::MatrixXd Q_bar;
    Eigen::MatrixXd R_bar;

    Eigen::MatrixXd H_qp;
    Eigen::MatrixXd g;

    Eigen::Vector3d desired_position;
    Eigen::Vector3d desired_velocity;
    

    rclcpp::Subscription<mission_interface::msg::SensorState>::SharedPtr measured_plant_state_sub;
    rclcpp::Subscription<mission_interface::msg::ReferenceState>::SharedPtr planner_state_sub;
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