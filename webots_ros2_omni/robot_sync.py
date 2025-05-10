import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class SyncJointTrajectory(Node):
    def __init__(self):
        super().__init__('sync_joint_trajectory')

        self.omni_action_client = ActionClient(
            self, FollowJointTrajectory, '/omni/omni_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.ur3_action_client = ActionClient(
            self, FollowJointTrajectory, '/ur3/ur_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for action servers...')
        self.omni_action_client.wait_for_server()
        self.ur3_action_client.wait_for_server()
        self.get_logger().info('Action servers available!')

        self.send_goals()

    def send_goals(self):
        omni_goal = FollowJointTrajectory.Goal()
        omni_goal.trajectory.joint_names = ['waist', 'shoulder', 'elbow', 'yaw', 'pitch', 'roll']
        omni_points = [
            JointTrajectoryPoint(positions=[0.5, 0.3, -0.2, 0.1, 0.0, -0.1], time_from_start=rclpy.duration.Duration(seconds=2).to_msg()),
            JointTrajectoryPoint(positions=[0.2, 0.1, 0.0, -0.1, 0.2, 0.3], time_from_start=rclpy.duration.Duration(seconds=4).to_msg()),
            JointTrajectoryPoint(positions=[-0.3, -0.2, 0.1, 0.0, -0.1, 0.5], time_from_start=rclpy.duration.Duration(seconds=6).to_msg())
        ]
        omni_goal.trajectory.points = omni_points

        ur3_goal = FollowJointTrajectory.Goal()
        ur3_goal.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        ur3_points = [
            JointTrajectoryPoint(positions=[0.5, -0.3, 0.2, -0.1, 0.0, 0.1], time_from_start=rclpy.duration.Duration(seconds=2).to_msg()),
            JointTrajectoryPoint(positions=[0.3, 0.1, -0.2, 0.0, 0.2, -0.3], time_from_start=rclpy.duration.Duration(seconds=4).to_msg()),
            JointTrajectoryPoint(positions=[-0.2, 0.0, 0.1, 0.3, -0.1, 0.5], time_from_start=rclpy.duration.Duration(seconds=6).to_msg())
        ]
        ur3_goal.trajectory.points = ur3_points

        self.get_logger().info('Sending complex goals to Omni and UR3...')
        omni_future = self.omni_action_client.send_goal_async(omni_goal)
        ur3_future = self.ur3_action_client.send_goal_async(ur3_goal)

        # Wait for results
        omni_future.add_done_callback(self.omni_goal_response_callback)
        ur3_future.add_done_callback(self.ur3_goal_response_callback)

    def omni_goal_response_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info('Omni goal accepted.')
        else:
            self.get_logger().error('Omni goal rejected.')

    def ur3_goal_response_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info('UR3 goal accepted.')
        else:
            self.get_logger().error('UR3 goal rejected.')

def main(args=None):
    rclpy.init(args=args)
    sync_joint_trajectory = SyncJointTrajectory()
    rclpy.spin(sync_joint_trajectory)
    sync_joint_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
