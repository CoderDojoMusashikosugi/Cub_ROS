import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import yaml
import math

class WaypointActionClient(Node):
    def __init__(self, yaml_file):
        super().__init__('waypoint_action_client')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.yaml_file = yaml_file

    def send_goal(self):
        # YAMLファイルを読み込み
        with open(self.yaml_file, 'r') as file:
            coordinates = yaml.safe_load(file)['coordinates']

        # Waypointsのリストを作成
        waypoints = []
        for coord in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = coord['x']
            pose.pose.position.y = coord['y']
            pose.pose.position.z = 0.0
            # yawをクォータニオンに変換
            qz, qw = math.sin(math.radians(coord['yaw']) / 2), math.cos(math.radians(coord['yaw']) / 2)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            waypoints.append(pose)

        # アクションゴールの設定
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        # サーバーが起動するまで待機してから送信
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoints followed successfully')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    yaml_file = '../routes/coordinates.yaml'  # YAMLファイルのパス
    waypoint_action_client = WaypointActionClient(yaml_file)
    waypoint_action_client.send_goal()
    rclpy.spin(waypoint_action_client)

if __name__ == '__main__':
    main()
