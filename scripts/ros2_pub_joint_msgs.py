import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        point1.time_from_start.sec = 1
        point2 = JointTrajectoryPoint()
        point2.positions = [1.0, -1.0, 1.0, 0.5, 1.0, -0.5]
        point2.time_from_start.sec = 3
        traj.points = [point1, point2]

        self.pub.publish(traj)
        self.get_logger().info("Published JointTrajectory")
        self.timer.cancel()  # publish only once

rclpy.init()
node = TrajectoryPublisher()
rclpy.spin(node)
