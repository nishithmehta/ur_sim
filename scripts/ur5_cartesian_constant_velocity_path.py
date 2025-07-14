#!/usr/bin/env python3
"""
ur5_cartesian_constant_velocity_path.py

Generate a constant-velocity Cartesian path for UR5, solve IK using ikpy,
publish as JointTrajectory, and plot joint velocities.

Requirements:
- pip install ikpy matplotlib
- ROS 2 environment sourced
"""

import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import URDFLink
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# --------- Define UR5 DH chain for ikpy ----------
ur5_chain = Chain(name='ur5', links=[
    URDFLink(
        name="base_link",
        origin_translation=[0, 0, 0.089159],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="shoulder_link",
        origin_translation=[0, 0.13585, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="upper_arm_link",
        origin_translation=[0.425, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="forearm_link",
        origin_translation=[0.39225, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="wrist_1_link",
        origin_translation=[0, 0.10915, 0],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0],
    ),
    URDFLink(
        name="wrist_2_link",
        origin_translation=[0, 0.09465, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    URDFLink(
        name="wrist_3_link",
        origin_translation=[0, 0.0823, 0],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0],
    )
])

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

# --------- 1. Define Cartesian Waypoints (XY Plane Example) ----------
waypoints = np.array([
    [0.40, 0.10, 0.20],  # Start
    [0.50, 0.10, 0.20],  # Move in X
    [0.50, 0.20, 0.20],  # Move in Y
])

# --------- 2. Interpolate Path for Constant-Velocity -----------
def interpolate_path(waypoints, ds=0.01):
    path = []
    for i in range(len(waypoints)-1):
        p0 = waypoints[i]
        p1 = waypoints[i+1]
        dist = np.linalg.norm(p1 - p0)
        n_points = max(int(np.ceil(dist / ds)), 1)
        for j in range(n_points):
            alpha = j / n_points
            point = (1 - alpha) * p0 + alpha * p1
            path.append(point)
    path.append(waypoints[-1])
    return np.array(path)

cartesian_path = interpolate_path(waypoints, ds=0.01)

def ik_for_path(cartesian_path):
    q_list = []
    q0 = [0, -1.57, 1.57, 0, 1.57, 0]  # 6 joint values
    full_q0 = [0] + q0                 # 7 values
    for pose in cartesian_path:
        ik_sol = ur5_chain.inverse_kinematics(
            target_position=pose,
            initial_position=full_q0    # 7 values!
        )
        q = ik_sol[1:7]
        if np.any(np.isnan(q)):
            print("IK failure for pose:", pose)
            q = q_list[-1] if q_list else q0  # fallback to last
        q_list.append(q)
        full_q0 = [0] + list(q)        # update full_q0 for next iteration
    return np.array(q_list)

joint_path = ik_for_path(cartesian_path)

# --------- 4. Plot Joint Velocities ----------
dt = 0.02  # 20ms for constant velocity (adjust for slower/faster EE)
joint_velocities = np.diff(joint_path, axis=0) / dt

plt.figure(figsize=(10,5))
for i in range(joint_velocities.shape[1]):
    plt.plot(joint_velocities[:, i], label=f'Joint {i+1}')
plt.xlabel('Step')
plt.ylabel('Joint velocity (rad/s)')
plt.legend()
plt.title('UR5 Joint Velocities Along Cartesian Path')
plt.tight_layout()
plt.show()

# --------- 5. ROS2 Node to Send Trajectory (waits for current state) ----------
class TrajectorySender(Node):
    def __init__(self, joint_path, dt):
        super().__init__('ur5_cartesian_path_sender')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.joint_path = joint_path
        self.dt = dt
        self.current_positions = None
        self.sent = False

    def joint_state_callback(self, msg):
        if self.sent:
            return  # Only send once!
        joint_map = dict(zip(msg.name, msg.position))
        if all(j in joint_map for j in JOINT_NAMES):
            current_pos = [joint_map[j] for j in JOINT_NAMES]
            self.send_trajectory(current_pos)
            self.sent = True

    def send_trajectory(self, start_pos):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        # Initial hold points at current position (t=0.0, t=0.2)
        pt0 = JointTrajectoryPoint()
        pt0.positions = start_pos
        pt0.velocities = [0.0] * len(start_pos)
        pt0.time_from_start.sec = 0
        pt0.time_from_start.nanosec = 0
        traj.points.append(pt0)

        pt_hold = JointTrajectoryPoint()
        pt_hold.positions = start_pos
        pt_hold.velocities = [0.0] * len(start_pos)
        pt_hold.time_from_start.sec = 0
        pt_hold.time_from_start.nanosec = int(0.2 * 1e9)
        traj.points.append(pt_hold)

        # Blend from start_pos to first IK pose
        blend_steps = 30
        blend_time = 1.0
        q_start = self.joint_path[0]
        for i in range(blend_steps):
            alpha = (i + 1) / blend_steps
            q_blend = (1 - alpha) * np.array(start_pos) + alpha * np.array(q_start)
            pt = JointTrajectoryPoint()
            t = 0.2 + (i + 1) * (blend_time / blend_steps)
            pt.positions = q_blend.tolist()
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            traj.points.append(pt)

        # The rest of the planned IK path (after blend)
        time_from_start = 0.2 + blend_time + self.dt
        for q in self.joint_path:
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.time_from_start.sec = int(time_from_start)
            pt.time_from_start.nanosec = int((time_from_start % 1) * 1e9)
            traj.points.append(pt)
            time_from_start += self.dt

        self.pub.publish(traj)
        self.get_logger().info(f"Published Cartesian constant-velocity path with blend to IK start ({len(self.joint_path)} IK points).")


def ros_main(joint_path, dt):
    rclpy.init()
    node = TrajectorySender(joint_path, dt)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# --------- 6. Main Glue ---------
if __name__ == "__main__":
    print(f"Generated {len(cartesian_path)} Cartesian path points.")
    print(f"Generated {len(joint_path)} IK joint path points.")
    print("Plotting joint velocities...")
    ros_main(joint_path, dt)
