# ROS 2 Autonomous Differential Drive Robot

Autonomous differential drive robot built from scratch, featuring custom control loops, odometry, distributed ROS 2 architecture, and a real-time web interface. The system uses an ESP32 as a low-level hardware controller integrated with a PC-based high-level navigation node via micro-ROS.

> **Note:** The source code and this README are in English, but the detailed technical documentation (`Informe_laboratorio_de_robotica.pdf`) located in the `docs/` folder is written in Spanish.

## Hardware & Electromechanical Design

* **Chassis:** Custom-built lightweight structure with a thematic design, integrating an analog parking brake that physically locks the wheels when powered off.
* **Sensors:** 3x HC-SR04 Ultrasonic sensors (Front, Left, Right) with non-blocking scheduled reading (100ms intervals) and 25ms hardware timeouts.
* **Actuators:** 2x DC Motors with quadrature encoders.
* **Electronics:** ESP32 microcontroller, integrated voltmeter for battery monitoring, and isolated power switches for logic and motors.

## Core Robotics Capabilities

1. **Custom PID Control:** Generic PID implementation with anti-windup for independent wheel speed control.
2. **Odometry & Kinematics:** 
   * Inverse kinematics for differential drive (linear and angular velocity to independent wheel speeds).
   * Midpoint approximation (Runge-Kutta 2nd order) for precise state estimation (X, Y, Theta) updated at 100Hz.
3. **Follow the Carrot Navigation:** Waypoint navigation algorithm relying on Euclidean distance and bearing calculations. Prioritizes pure rotation for high angular errors and blended forward/angular velocity for path following.
4. **Reactive Safety:** Hardware-level override that stops the robot if an obstacle is detected within 15 cm, bypassing any external navigation commands.

## Distributed Architecture (micro-ROS)

The system is decoupled using ROS 2 (UDP over WiFi):
* **ESP32 (Low-Level):** Runs micro-ROS, computing local odometry and publishing to `/robot_pose` at 20Hz. Subscribes to `/cmd_vel` to execute velocities.
* **PC Node (High-Level):** C++ node (`follow_carrot_node`) handling trajectory planning and waypoint execution, calculating velocity commands based on the robot's pose.

## Web Interface (SPA) & Telemetry

A Single Page Application served by the ESP32 via WebSockets, acting as a unified command center:
* **Teleoperation:** HTML5 virtual joystick and native Gamepad API integration for DualShock 4 control.
* **Real-Time Visualization (Weak SLAM):** HTML5 Canvas plots the robot's odometry trail and projects sonar readings into a global coordinate system, generating a real-time 2D occupancy grid.
* **Tactile Navigation (Finger Follow):** Clicking any point on the generated map translates screen pixels to real-world coordinates, sending a Go-To command for autonomous navigation.
* **Mission Planner:** Dynamic UI to input exact coordinates, radii for circular trajectories, or sequential waypoints.

## Repository Structure

```text
├── src/
│   ├── firmware/             # ESP32 C++/Arduino code (PID, Odometry, micro-ROS setup)
│   └── ros2_ws/              # ROS 2 workspace (C++ Follow Carrot node)
├── web/                      # HTML, CSS, and JS (Canvas, WebSockets, Gamepad API)
└── docs/
    ├── Informe_laboratorio_de_robotica.pdf # Technical specification (ES)
    └── images/               # Robot photos and UI screenshots
