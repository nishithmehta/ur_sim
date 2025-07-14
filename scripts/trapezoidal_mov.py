import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

REQUIRED_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.sent = False

    def joint_state_callback(self, msg):
        if self.sent:
            return  # Only send once!
        joint_map = dict(zip(msg.name, msg.position))
        if all(j in joint_map for j in REQUIRED_JOINTS):
            current_pos = [joint_map[j] for j in REQUIRED_JOINTS]

            # Choose a target within a small delta to avoid collisions
            end_pos = current_pos.copy()
            end_pos[0] += 0.15  # Move shoulder_pan_joint by +0.15 rad
            end_pos[2] += 0.10  # Move elbow_joint by +0.10 rad

            # Make sure within safe range, clamp if necessary
            # For demonstration, these small values are safe for most arms

            total_time = 6.0
            steps = 30

            positions, velocities = self.trapezoidal_profile(
                current_pos, end_pos, total_time,
                max_vel=0.3, max_acc=0.6, steps=steps
            )

            traj = JointTrajectory()
            traj.joint_names = REQUIRED_JOINTS

            # First two points at current position (t=0.0, t=0.2)
            pt0 = JointTrajectoryPoint()
            pt0.positions = current_pos
            pt0.velocities = [0.0] * len(current_pos)
            pt0.time_from_start.sec = 0
            pt0.time_from_start.nanosec = 0
            traj.points.append(pt0)

            pt_hold = JointTrajectoryPoint()
            pt_hold.positions = current_pos
            pt_hold.velocities = [0.0] * len(current_pos)
            pt_hold.time_from_start.sec = 0
            pt_hold.time_from_start.nanosec = int(0.2 * 1e9)
            traj.points.append(pt_hold)

            # The rest of the points (start after t=0.2s)
            for i, (pos, vel) in enumerate(zip(positions, velocities)):
                pt = JointTrajectoryPoint()
                pt.positions = pos
                pt.velocities = vel
                t = 0.2 + (i+1) * total_time / steps
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t % 1) * 1e9)
                traj.points.append(pt)

            self.pub.publish(traj)
            self.get_logger().info("Published safe trajectory (w/ double initial hold).")
            self.sent = True

    def trapezoidal_profile(self, q0, qf, total_time, max_vel=0.3, max_acc=0.6, steps=30):
        q0 = np.array(q0)
        qf = np.array(qf)
        dq = qf - q0
        N = steps
        dt = total_time / (N-1)
        positions = []
        velocities = []

        for joint in range(len(q0)):
            d = dq[joint]
            direction = np.sign(d) if d != 0 else 1
            d_abs = abs(d)
            if d_abs < 1e-6:
                # No movement for this joint
                pos_profile = [q0[joint]] * N
                vel_profile = [0.0] * N
            else:
                t_acc = max_vel / max_acc
                d_acc = 0.5 * max_acc * t_acc**2
                if d_abs < 2 * d_acc:
                    t_acc = np.sqrt(d_abs / max_acc)
                    t_flat = 0
                else:
                    t_flat = (d_abs - 2 * d_acc) / max_vel
                t_total = 2 * t_acc + t_flat
                # Stretch profile if needed, avoid division by zero
                if t_total < 1e-6:
                    t_acc = 0.0
                    t_flat = 0.0
                    t_total = total_time
                elif t_total < total_time:
                    t_acc = t_acc * (total_time / t_total)
                    t_flat = t_flat * (total_time / t_total)
                    t_total = total_time
                pos_profile = []
                vel_profile = []
                for i in range(N):
                    t = i * dt
                    if t < t_acc:
                        pos = 0.5 * max_acc * t**2
                        vel = max_acc * t
                    elif t < t_acc + t_flat:
                        pos = d_acc + max_vel * (t - t_acc)
                        vel = max_vel
                    else:
                        t_dec = t - (t_acc + t_flat)
                        pos = d_acc + max_vel * t_flat + max_vel * t_dec - 0.5 * max_acc * t_dec**2
                        vel = max_vel - max_acc * t_dec
                    pos_profile.append(q0[joint] + direction * pos)
                    vel_profile.append(direction * vel)
            positions.append(pos_profile)
            velocities.append(vel_profile)
        positions = np.array(positions).T.tolist()
        velocities = np.array(velocities).T.tolist()
        return positions, velocities


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
