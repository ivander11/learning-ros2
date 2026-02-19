/**<br>
 * Minimal C++ publisher example for ROS 2.
 * This example demonstrates how to create a simple publisher node in C++.
 * It publishes a string message every 500 milliseconds on the topic "/cpp_example_topic".
 */

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ client library
#include "std_msgs/msg/string.hpp" // Standard message type for string data

using namespace std::chrono_literals; // For using time literals like 500ms

class MinimalCppPublisher : public rclcpp::Node
{
public:
    MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0)
    {
        // Create a publisher that publishes String messages on the "topic" topic
        publisher_ = this->create_publisher<std_msgs::msg::String>( 
            "/cpp_example_topic", 10);

        // Create a timer that calls the timer_callback function every 500 milliseconds
        timer_ = this->create_wall_timer(500ms, 
            std::bind(&MinimalCppPublisher::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Publishing at 2 Hz");
    }
    
    void timerCallback()
    {
        // Create a new String message
        auto message = std_msgs::msg::String();

        // Set the data field of the message to "Hello, world!" followed by the count
        message.data = "Hello, world! " + std::to_string(count_++);
        
        // Publish the message
        publisher_->publish(message);   
    }

private:
    size_t count_; // Counter to keep track of the number of messages published
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publisher object
    rclcpp::TimerBase::SharedPtr timer_; // Timer object
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the MinimalCppPublisher node
    auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>();

    // Spin the node to keep it alive and processing callbacks
    rclcpp::spin(minimal_cpp_publisher_node);

    rclcpp::shutdown();
    
    return 0;
}