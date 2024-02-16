#!/usr/bin/env python3
import time
import rclpy
import asyncio
import threading
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from tf2_ros import TransformBroadcaster
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# Test with:
# ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}}}, behavior_tree: 'bt_xml_file'}" --feedback

class BasicNavigatorSimulator(Node):
    def __init__(self):
        super().__init__("tt_umpire_basic_navigator_simulator")
        self.mutex = threading.Lock()
        self.current_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.pose_step = 0.1
        self.subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            1
        )
        self.amcl_get_state_service = self.create_service(
            GetState,
            "/amcl/get_state",
            self.mock_get_state_callback
        )
        self.bt_get_state_service = self.create_service(
            GetState,
            "/bt_navigator/get_state",
            self.mock_get_state_callback
        )
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, "amcl_pose", 10)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self.execute_callback,
            #callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # Set up publisher and subscriber
        self.tf_broadcaster = TransformBroadcaster(self)

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        asyncio.run(self.execute_callback(None))

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def mock_get_state_callback(self, request, response):
        response.current_state.id = 3
        response.current_state.label = 'active'
        return response

    def goal_callback(self, goal_request):
        with self.mutex:
            self.get_logger().info('Received goal request')
            self.goal_pose = goal_request.pose
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    async def execute_callback(self, goal_handle):
        elapsed_time = 0.0
        with self.mutex:
            while self.goal_pose != self.current_pose:
                if goal_handle and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result = NavigateToPose.Result()
                    result.error_code = 666
                    return result
                self.current_pose.pose.orientation = self.goal_pose.pose.orientation

                vector = Vector3()
                vector.x = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
                vector.y = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
                vector.z = self.goal_pose.pose.position.z - self.current_pose.pose.position.z
                
                # Normalize the vector to unit length
                distance = (vector.x ** 2 + vector.y ** 2 + vector.z ** 2) ** 0.5
                vector.x /= distance
                vector.y /= distance
                vector.z /= distance
                if distance <= self.pose_step:
                    self.current_pose = self.goal_pose

                # Move self.current_pose closer to self.goal_pose by pose_step meters
                self.current_pose.pose.position.x += self.pose_step * vector.x
                self.current_pose.pose.position.y += self.pose_step * vector.y
                self.current_pose.pose.position.z += self.pose_step * vector.z

                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = "map"
                transform.child_frame_id = "odom"
                self.tf_broadcaster.sendTransform(transform)
                transform.header.frame_id = "odom"
                transform.child_frame_id = "base_link"
                transform.transform.translation.x = self.current_pose.pose.position.x
                transform.transform.translation.y = self.current_pose.pose.position.y
                transform.transform.translation.z = self.current_pose.pose.position.z
                transform.transform.rotation.x = self.current_pose.pose.orientation.x
                transform.transform.rotation.y = self.current_pose.pose.orientation.y
                transform.transform.rotation.z = self.current_pose.pose.orientation.z
                transform.transform.rotation.w = self.current_pose.pose.orientation.w
                self.tf_broadcaster.sendTransform(transform)

                amcl_pose = PoseWithCovarianceStamped()
                amcl_pose.header.stamp = self.get_clock().now().to_msg()
                amcl_pose.header.frame_id = "map"
                amcl_pose.pose.pose = self.current_pose.pose
                self.amcl_pub.publish(amcl_pose)

                elapsed_time += 1
                if goal_handle:
                    feedback = NavigateToPose.Feedback()
                    feedback.current_pose = self.current_pose
                    feedback.distance_remaining = distance
                    feedback.navigation_time = Duration(sec=int(elapsed_time))
                    feedback.estimated_time_remaining = Duration(sec=int(distance*self.pose_step))
                    goal_handle.publish_feedback(feedback)

                time.sleep(1)

            if goal_handle:
                goal_handle.succeed()
            return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    basic_navigator_simulator = BasicNavigatorSimulator()
    rclpy.spin(basic_navigator_simulator, executor=executor)
    basic_navigator_simulator.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
