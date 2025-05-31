# Autonomous-obstacle-avoidance-in-Husarion-ROSbot

## Overview

This project implements an autonomous obstacle avoidance system for the Husarion ROSbot 2.0, enabling the robot to navigate from a starting position to a goal location while autonomously avoiding obstacles using sensor data and advanced path planning algorithms. [code available in the report]

## Project Description

The system addresses the growing need for autonomous navigation in robotics, with applications ranging from industrial automation to autonomous vehicles. By 2035, autonomous driving could create $300-400 billion in revenue, making robust obstacle avoidance systems crucial for the future of mobility.

This implementation uses the **Tangent Bug Algorithm** with LiDAR sensor integration to provide reliable, real-time obstacle detection and path planning capabilities.

## Hardware Specifications

### Husarion ROSbot 2.0
- **Controller**: Husarion CORE2-ROS robot controller
- **Platform**: 4-wheeled mobile platform with DC motors and encoders
- **Frame**: Aluminum construction
- **Maximum Speed**: 1.25 m/s translational velocity

### Sensors
- **RPLIDAR A2**: Laser scanner (360° scanning, 12m range, 10Hz frequency)
- **Orbbec Astra RGBD Camera**: RGB camera with depth perception
- **MPU 9250**: 9-axis inertial sensor (accelerometer + gyroscope)
- **ToF Sensors**: Four Time-of-Flight infrared distance sensors (200cm range)
- **Wi-Fi Antenna**: Wireless communication capabilities

### Computing Platform
- **Microcontroller**: ASUS Tinker Board
- **Memory**: 2GB RAM
- **Processor**: Cortex A17 Quadcore 1.8 GHz
- **OS**: Ubuntu Linux 20.04 LTS
- **Framework**: ROS Noetic

## Software Architecture

### Core Components

#### Navigation Algorithm: Tangent Bug
The system implements the **Tangent Bug Algorithm**, which improves upon the traditional Bug 2 algorithm by:
- Using sensor range information for better path planning
- Implementing motion-to-goal and boundary-following behaviors
- Minimizing heuristic distance to optimize path efficiency
- Handling complex obstacle configurations

#### ROS Node Structure
```
/stage (Stage Simulator)
├── /robot_0/cmd_vel (Velocity Commands)
├── /robot_0/odom (Odometry Data)
├── /robot_0/base_scan (Laser Scan Data)
├── /robot_0/base_pose_ground_truth (Ground Truth Position)
├── /tf (Transformation Tree)
├── /clock (Simulation Time)
└── /statistics (Performance Metrics)
```

### Key Python Modules

1. **frame.py**: Reference frame management and position tracking
2. **tangentbug.py**: Main navigation algorithm implementation
3. **actionServer.py**: Action server for goal management
4. **goal.py**: Goal position determination and communication
5. **testclient.py**: Goal client setup and status monitoring

## Installation and Setup

### Prerequisites
```bash
# ROS Noetic installation required
sudo apt update
sudo apt install ros-noetic-desktop-full

# Additional dependencies
sudo apt install ros-noetic-stage-ros
sudo apt install ros-noetic-navigation
sudo apt install python3-rospy
```

### Clone and Build
```bash
# Clone the repository
git clone [repository-url]
cd autonomous-rosbot-navigation

# Build the workspace
catkin_make
source devel/setup.bash
```

## Usage

### Launch the System
```bash
roslaunch as5831_rosbot as5831_rosbot.launch
```

This command initializes:
- Gazebo simulation environment
- ROSbot model with sensor integration
- Navigation algorithm nodes
- Goal management system

### Configuration Parameters
- **LiDAR Range**: 5 meters (optimized for indoor environments)
- **Scanning Frequency**: 10 Hz
- **Goal Position**: Default (3, 12) - configurable in goal.py
- **Obstacle Detection Threshold**: Adjustable discontinuity detection

## Algorithm Details

### Tangent Bug Implementation

#### Motion-to-Goal Behavior
- Direct movement toward goal when path is clear
- Continuous goal visibility assessment
- Optimal direction calculation using heuristic distance

#### Boundary-Following Behavior
- Obstacle circumnavigation when direct path is blocked
- Leave-point detection using distance criteria
- Smooth transition between behaviors

#### Key Features
- **Sensor Fusion**: Combines LiDAR data with odometry
- **Real-time Processing**: 10 Hz update rate for responsive navigation
- **Adaptive Planning**: Dynamic path adjustment based on sensor feedback
- **Collision Avoidance**: Priority-based obstacle avoidance system

## Testing Framework

### Test Cases

#### Case 1: Direct Line Navigation
- **Scenario**: Robot and goal on same line with obstacle
- **Behavior**: Motion-to-goal → Boundary-following → Motion-to-goal
- **Result**: ✅ Successful obstacle avoidance and goal reaching

#### Case 2: Non-Convex Obstacles
- **Scenario**: Complex obstacle shapes
- **Challenge**: Optimal circumnavigation path selection
- **Result**: ⚠️ Algorithm struggles with optimal approach definition

#### Case 3: Multiple Obstacles
- **Scenario**: Sequential obstacle navigation
- **Purpose**: Algorithm repeatability testing
- **Result**: ✅ Consistent behavior across multiple obstacles

#### Case 4: Goal Within Obstacle Region
- **Scenario**: Unreachable goal position
- **Behavior**: Boundary clinging and oscillation
- **Result**: ⚠️ Robot exhibits jittering behavior

## Performance Metrics

### Successful Navigation Criteria
- Obstacle detection accuracy: >95%
- Path planning efficiency: Optimized for time and distance
- Goal reaching precision: Within acceptable tolerance
- System stability: Consistent behavior across test scenarios

### Limitations Identified
- Difficulty with optimal path selection for complex obstacles
- Oscillation behavior when goal is within obstacle regions
- Performance dependency on LiDAR data quality

## Applications

### Industrial Robotics
- Warehouse automation
- Manufacturing floor navigation
- Inventory management systems

### Research and Development
- Algorithm prototyping
- Multi-robot coordination
- Autonomous system testing

### Educational Purposes
- Robotics curriculum integration
- Hands-on autonomous navigation learning
- Algorithm comparison studies

## Future Enhancements

### Planned Improvements
- **Advanced Path Planning**: Integration of A* or RRT algorithms
- **Multi-Robot Coordination**: Swarm intelligence implementation
- **Machine Learning**: Neural network-based obstacle prediction
- **Sensor Fusion Enhancement**: Integration of camera and IMU data

### Research Opportunities
- Dynamic obstacle handling
- Real-time map building (SLAM)
- Energy-efficient navigation
- Human-robot interaction in navigation

## Dependencies

### ROS Packages
```xml
<depend>rospy</depend>
<depend>tf</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<depend>actionlib</depend>
<depend>stage_ros</depend>
```

### Python Libraries
```python
import rospy
import tf
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt32
```

## Troubleshooting

### Common Issues
1. **LiDAR Data Noise**: Adjust max_range parameter in tangentbug.py
2. **Goal Not Reached**: Verify goal coordinates in goal.py
3. **Simulation Crashes**: Check Gazebo world file integrity
4. **Node Communication Errors**: Verify ROS master is running

### Debug Tools
- **rqt_graph**: Visualize node communication
- **rviz**: Monitor robot state and sensor data
- **rostopic echo**: Check message publishing
- **rosnode list**: Verify active nodes

## Contributing

### Development Guidelines
- Follow ROS coding standards
- Document algorithm modifications
- Test with multiple obstacle configurations
- Validate performance metrics

### Code Structure
```
src/
├── launch/
│   └── as5831_rosbot.launch
├── scripts/
│   ├── frame.py
│   ├── tangentbug.py
│   ├── actionServer.py
│   ├── goal.py
│   └── testclient.py
└── worlds/
    └── simulation_environment.world
```

## License

This project is developed for academic and research purposes at the University of Bath, Department of Electrical and Electronics Engineering.

## Author

**Aseem Saxena**  
Department of Electrical and Electronics Engineering  
University of Bath

## References

1. Choset, H.M. (2005). *Principles of robot motion: theory, algorithms, and implementation*
2. Van Breda, R. (2016). *Vector field histogram star obstacle avoidance system for multicopters*
3. McKinsey (2024). *The future of autonomous vehicles*
4. ROS Documentation: http://wiki.ros.org/Documentation

---

*For technical support or collaboration opportunities, please contact the project author*
