# SpiderPi Pro

English | [中文](https://github.com/Hiwonder/SpiderPi-Pro/blob/main/README_cn.md)

<p align="center">
  <img src="./sources/images/image.webp" alt="SpiderPi Pro Logo" width="400"/>
</p>

## Product Overview

Hiwonder SpiderPi Pro is an AI vision hexapod robot kit based on Raspberry Pi 5/Raspberry Pi 4B. Building upon the original SpiderPi hexapod robot, it adds a vision robotic arm, enabling more interesting AI creative applications such as object recognition and grasping, transportation and handling, intelligent sorting, and multi-robot control. It not only satisfies users' learning and verification needs for machine vision, hexapod gait, and robotic arm kinematics, but also provides a fast and convenient integration solution for secondary development of sensor applications and visual grasping.

## Official Resources

### Official Hiwonder
- **Official Website**: [https://www.hiwonder.net/](https://www.hiwonder.net/)
- **Product Page**: [https://www.hiwonder.com/products/spiderpi-pro](https://www.hiwonder.com/products/spiderpi-pro)
- **Official Documentation**: [https://docs.hiwonder.com/projects/SpiderPi_Pro/en/latest/](https://docs.hiwonder.com/projects/SpiderPi_Pro/en/latest/)
- **Technical Support**: support@hiwonder.com

## Key Features

### AI Vision Functions
- **Color Detection** - Advanced color recognition and tracking
- **Color Tracking** - Real-time color-based object tracking
- **Face Detection** - Comprehensive face recognition capabilities
- **AprilTag Detection** - Precision tag recognition for navigation
- **Visual Patrol** - Intelligent visual surveillance and monitoring
- **Object Recognition** - AI-powered object identification

### Advanced Applications
- **Intelligent Fetch** - Autonomous object recognition and grasping
- **Block Fetch** - Precise block manipulation and retrieval
- **Color Sorting** - Automated color-based object sorting
- **Shape Recognition** - Geometric shape detection and analysis
- **Intelligent Kick** - Ball kicking with visual feedback
- **Cruise Carry** - Autonomous navigation and transportation
- **Ball Orientation** - Ball tracking and positioning

### Motion Control
- **Hexapod Gait Control** - Advanced six-legged locomotion algorithms
- **Robotic Arm Kinematics** - Inverse kinematics for arm control
- **Action Groups** - Pre-programmed motion sequences
- **Remote Control** - Wireless control via joystick and network
- **Multi-Robot Control** - Coordinated control of multiple robots
- **Obstacle Avoidance** - Intelligent obstacle detection and avoidance

### Programming Interface
- **Python Programming** - Complete Python SDK
- **RPC Interface** - JSON-RPC remote calls
- **Video Stream** - Real-time MJPG video streaming
- **Joystick Support** - Game controller integration
- **Open Source** - Complete open-source platform for customization

## Hardware Configuration
- **Processor**: Raspberry Pi 5 or Raspberry Pi 4B
- **Structure**: Hexapod robot with integrated robotic arm
- **Servos**: High-precision digital servos
- **Vision System**: HD camera with pan-tilt mechanism
- **Sensors**: Ultrasonic sensor, camera
- **Communication**: WiFi, Bluetooth

## Project Structure

```
spiderpi_pro/
├── SpiderPi.py               # Main program entry
├── rpc_server.py            # RPC server
├── mjpg_Server.py           # Video stream server
├── joystick.py              # Joystick control
├── pick_action.py           # Grasping actions
├── action_group_dict.py     # Action group definitions
├── action_groups/           # Pre-programmed action sequences
├── functions/               # Function modules
│   ├── color_detect.py      # Color detection
│   ├── color_track.py       # Color tracking
│   ├── face_detect.py       # Face detection
│   ├── apriltag_detect.py   # AprilTag detection
│   ├── visual_patrol.py     # Visual patrol
│   ├── avoidance.py         # Obstacle avoidance
│   ├── remote_control.py    # Remote control
│   ├── robot_dance.py       # Dance performance
│   └── multi_control_*.py   # Multi-robot control
├── advanced/                # Advanced applications
│   ├── intelligent_fetch.py # Intelligent grasping
│   ├── block_fetch.py       # Block manipulation
│   ├── color_sorting.py     # Color sorting
│   ├── shape_recognition.py # Shape recognition
│   ├── intelligent_kick.py  # Ball kicking
│   ├── cruise_carry.py      # Cruise and carry
│   └── ball_orientation.py  # Ball orientation
├── kinematic_routines/      # Kinematics algorithms
├── spiderpi_sdk/           # Hardware control SDK
├── config/                 # Configuration files
└── sources/                # Resources and documentation
```

## Version Information
- **Current Version**: SpiderPi Pro v1.0.0
- **Supported Platform**: Raspberry Pi 5, Raspberry Pi 4B

### Related Technologies
- [OpenCV](https://opencv.org/) - Computer Vision Library
- [Python](https://www.python.org/) - Programming Language

---

**Note**: This program is pre-installed on the SpiderPi Pro robot system and can be run directly. For detailed tutorials, please refer to the [Official Documentation](https://docs.hiwonder.com/projects/SpiderPi_Pro/en/latest/).
