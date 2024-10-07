#ifndef ARM_SERVICE_CLIENT_HPP_
#define ARM_SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ArmServiceClient : public rclcpp::Node {
public:
    ArmServiceClient();

    bool armVehicle();
    bool setGuidedMode();
    void moveForward();

private:
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
};

#endif  // ARM_SERVICE_CLIENT_HPP_

