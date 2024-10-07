#include "bluerov2/ArmServiceClient.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArmServiceClient>();

    // Arm the vehicle
    if (node->armVehicle()) {
        RCLCPP_INFO(node->get_logger(), "Vehicle is armed, setting to GUIDED mode...");

        // Set to GUIDED mode
        if (node->setGuidedMode()) {
            RCLCPP_INFO(node->get_logger(), "Vehicle is in GUIDED mode, moving forward...");
            node->moveForward();
        } else {
            RCLCPP_INFO(node->get_logger(), "Failed to set vehicle to GUIDED mode.");
        }
    } else {
        RCLCPP_INFO(node->get_logger(), "Failed to arm the vehicle.");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

