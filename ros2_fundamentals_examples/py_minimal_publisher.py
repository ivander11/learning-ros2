#! /usr/bin/env python3

"""
Description:
    This ROS2 node publishes a "Hello World" string message at a fixed frequency.
    
    Publishing Topics:
        /py_example_topic (std_msgs/String)

    Subscription Topics:
        None

--------
Author: Ivander Sugiarta Halim
Date: 09/02/2026
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPyPublisher(Node):
    """
    Create a minimal publisher node.
    """

    def __init__(self):
        """
        Create a custom node class for publishing messages to a topic.
        """
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_py_publisher')
        
        # Create a publisher for the topic 'py_example_topic'
        # Queue size is 10 to buffer messages if the network is busy
        self.publisher_ = self.create_publisher(String, '/py_example_topic', 10)
        
        # Create a timer that triggers the callback every 1.0 second
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize a counter variable to track message ID
        self.i = 0

    def timer_callback(self):
        """
        Callback function executed periodically by the timer.
        """
        # Create a new String message object
        msg = String()
        msg.data = 'Hello, ROS2! %d' % self.i
        
        # Publish the message to the topic
        self.publisher_.publish(msg)
        
        # Log the data to the console for debugging
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        # Increment the counter for the next cycle
        self.i += 1

def main(args=None):
    """
    Main function to initialize the ROS2 node and start publishing messages.
    """
    
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPyPublisher()

    # Keep the node running and processing callbacks (blocking)
    rclpy.spin(minimal_publisher)

    # Clean up and destroy the node explicitly
    minimal_publisher.destroy_node()
    
    # Shutdown the ROS2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Execute the main function when the script is run directly
    main()