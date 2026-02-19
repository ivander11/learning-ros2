# Learning ROS 2

This repository contains my personal learning journey with **ROS 2 Jazzy**.

## What I'm Learning

This package helps me understand ROS 2 fundamentals through hands-on practice:
- Creating ROS 2 nodes
- Topic-based communication between nodes
- Publisher and Subscriber patterns
- Implementing nodes in both Python and C++

## Tech Stack
- **OS**: Ubuntu 24.04 LTS
- **ROS 2 Distro**: Jazzy Jalisco

## Package Contents

### C++ Examples
| Executable | Description |
|------------|-------------|
| `cpp_minimal_publisher` | Publishes string messages every 500ms |
| `cpp_minimal_subscriber` | Subscribes to and displays messages |

### Python Examples
| Executable | Description |
|------------|-------------|
| `py_minimal_publisher.py` | Python version of the publisher |
| `py_minimal_subscriber.py` | Python version of the subscriber |

## Quick Start

### Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_fundamentals_examples
source install/setup.bash
```

### Run the Examples

**Terminal 1 - Publisher:**
```bash
ros2 run ros2_fundamentals_examples cpp_minimal_publisher
```

**Terminal 2 - Subscriber:**
```bash
ros2 run ros2_fundamentals_examples cpp_minimal_subscriber
```

### Verify Communication
```bash
# List active topics
ros2 topic list

# View published messages
ros2 topic echo /cpp_example_topic
```

## Learning Resources
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [Automatic Addison Tutorials](https://automaticaddison.com/tutorials/#Jazzy)

