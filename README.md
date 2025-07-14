# 6-DOF Robotic Arm Simulation (UR5)

**Tools:** ROS 2, Gazebo, RViz, MoveIt 2

## Overview

This project demonstrates the simulation and visualization of a 6-DOF robotic arm (**UR5**) in Gazebo and RViz using the official Universal Robots ROS 2 packages.

---

## 1. Prerequisites

* Ubuntu 22.04
* ROS 2 Humble (recommended)
* Gazebo (compatible with ROS 2)
* colcon build tool

---

## 2. Installation & Setup

### Clone Required Repositories

```bash
# Create and enter workspace
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src

# Clone Universal Robots packages (description + Gazebo sim)
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git

# Clone your custom package 
git clone https://github.com/nishithmehta/ur_sim
```

### Install Dependencies

```bash
cd ~/ur_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 3. How to Run the Simulation

### Launch Gazebo Simulation

```bash
ros2 launch ur_sim gazebo_ur5.launch.py
```

* Spawns the UR5 robot into the Gazebo simulation environment.

### Visualize in RViz

```bash
ros2 launch ur_sim rviz_ur5.launch.py
```

* Opens RViz and visualizes the UR5 robot state and joint positions.

### Launch MoveIt 2 for Motion Planning

```bash
ros2 launch ur_sim moveit_ur5.launch.py
```

* Starts MoveIt 2 for advanced motion planning and execution with the UR5 arm.

---

## 4. Testing Robot Movement with Scripts

After launching the Gazebo simulation, you can run the test scripts in the `scripts/` folder to command and test the UR5 robot’s movement.

### Example Usage

**Open a new terminal, source your workspace, and run:**

```bash
cd ~/ur_ws
source install/setup.bash
cd src/ur_sim/scripts
```

#### 1. Publish a sample joint trajectory

```bash
python3 ros2_pub_joint_msgs.py
```

*Publishes a short trajectory to the robot’s joint controllers for a basic test.*

#### 2. Send a smooth trajectory with trapezoidal velocity profile

```bash
python3 trapezoidal_mov.py
```

*Moves the arm a small, safe amount with a trapezoidal velocity profile, avoiding collisions.*

#### 3. Execute a Cartesian path at constant velocity (requires `ikpy` and `matplotlib`)

```bash
pip install ikpy matplotlib   # (if not already installed)
python3 ur5_cartesian_constant_velocity_path.py
```

*Generates a Cartesian path, solves inverse kinematics, sends a smooth trajectory to the arm, and plots joint velocities.*

---

### Scripts Overview

| Script Name                               | Description                                                                          |
| ----------------------------------------- | ------------------------------------------------------------------------------------ |
| `ros2_pub_joint_msgs.py`                  | Publishes a short, fixed joint trajectory for initial testing.                       |
| `trapezoidal_mov.py`                      | Sends a safe, smooth trajectory with a trapezoidal velocity profile to the robot.    |
| `ur5_cartesian_constant_velocity_path.py` | Generates a Cartesian path and commands the robot to follow it at constant velocity. |

---

### Tip:

Make sure the Gazebo simulation and the `/joint_trajectory_controller/joint_trajectory` topic are running before executing these scripts!

---

## 5. Project Structure

```
ur_ws/src/
├── Universal_Robots_ROS2_Description/
├── Universal_Robots_ROS2_Gazebo_Simulation/
├── ur_sim/
│   ├── launch/
│   │   ├── gazebo_ur5.launch.py
│   │   ├── rviz_ur5.launch.py
│   │   └── moveit_ur5.launch.py
│   ├── scripts/
│   │   ├── ros2_pub_joint_msgs.py
│   │   ├── trapezoidal_mov.py
│   │   └── ur5_cartesian_constant_velocity_path.py
│   ├── moveit_2_config/
│   │   └── ...    # MoveIt 2 configuration files
│   ├── videos/
│   │   └── ...    # Screencast videos for the assignment
│   └── ...
└── ...
```

---

## 6. Deliverables

* **Gazebo Demo:** Working simulation of the UR5 in Gazebo.
* **RViz Demo:** Visualization and joint control of the UR5 in RViz.
* **MoveIt 2 Demo:** Motion planning and execution using MoveIt 2 (`moveit_ur5.launch.py`).
* **Demo Videos:** 1–2 minute screen recordings (placed in the `videos/` folder) showing Gazebo, RViz, and MoveIt 2 in action.

---

## 7. References

* [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
* [Universal Robots ROS2 Gazebo Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation)
* [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
* [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/index.html)

---

## 8. Contact

For doubts or issues, reach out to **\[Nishith Mehta]**
Email: **\[[nishithm24@gmail.com](mailto:nishithm24@gmail.com)]**
