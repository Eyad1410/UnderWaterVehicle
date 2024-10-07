#include "bluerov2/ArmServiceClient.hpp"


ArmServiceClient::ArmServiceClient() : Node("arm_and_move_client") {
    // Create clients to arm the vehicle and set mode
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // Create publisher to send velocity commands
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
}

bool ArmServiceClient::armVehicle() {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    if (!arm_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Service /mavros/cmd/arming not available.");
        return false;
    }

    auto future = arm_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully.");
            return true;
        }
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to arm the vehicle.");
    return false;
}

bool ArmServiceClient::setGuidedMode() {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "GUIDED";

    if (!set_mode_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Service /mavros/set_mode not available.");
        return false;
    }

    auto future = set_mode_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "GUIDED mode set successfully.");
            return true;
        }
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to set GUIDED mode.");
    return false;
}

void ArmServiceClient::moveForward() {
    RCLCPP_INFO(this->get_logger(), "Moving the vehicle forward.");

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.5;  // Move forward with 0.5 m/s speed
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    vel_publisher_->publish(twist_msg);
}

