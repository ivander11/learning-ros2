/**
 * Minimal C++ subscriber example for ROS 2.
 * This example demonstrates how to create a simple subscriber node in C++.
 * It subscribes to a string message on the topic "/cpp_example_topic".
 */
 
#include "rclcpp/rclcpp.hpp" // ROS 2 C++ client library
#include "std_msgs/msg/string.hpp"  // Standard message type for string data

using std::placeholders::_1; // Placeholder for callback function

class MinimalCppSubscriber : public rclcpp::Node
{
public:
    MinimalCppSubscriber() : Node("minimal_cpp_subscriber")
    {
        // Create a subscriber that listens to String messages on the ""
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/cpp_example_topic", 
            10, 
            std::bind(
                &MinimalCppSubscriber::topicCallback, 
                this,
                _1
            )
        );
    }
    void topicCallback(const std_msgs::msg::String::SharedPtr msg) const
    {
        // Log the received message to the console
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

private:
    // Member variable
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; 
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the MinimalCppSubscriber node
    auto minimal_cpp_subscriber_node = std::make_shared<MinimalCppSubscriber>();

    // Spin the node to keep it alive and processing callbacks
    rclcpp::spin(minimal_cpp_subscriber_node);

    rclcpp::shutdown();

    return 0;
}