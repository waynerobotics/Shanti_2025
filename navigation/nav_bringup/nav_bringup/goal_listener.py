#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState

class GoalListener(Node):
    def __init__(self):
        super().__init__('goal_listener')

        # Create an action client for the waypoint follower
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Create a subscription to listen for goal pose from RViz
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
            
        # Parameter to control whether the robot waits at waypoints
        self.declare_parameter('wait_at_waypoints', False)
        self.declare_parameter('wait_duration', 0.0)
        
        # Initialize state
        self.current_goal = None
        self.awaiting_result = False
        
        self.get_logger().info('Waiting for waypoint follower action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Waypoint follower action server is available!')
        
        # Check if the nav2 stack is active before accepting goals
        self.create_timer(1.0, self.check_nav2_ready)
        self.nav2_ready = False

    def check_nav2_ready(self):
        """Check if Nav2 stack is active and ready to receive goals"""
        try:
            # Create a client to check the state of bt_navigator 
            client = self.create_client(GetState, 'bt_navigator/get_state')
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Navigation stack is active and ready')
                self.nav2_ready = True
                return True
        except Exception as e:
            self.get_logger().warn(f'Navigation stack not yet ready: {str(e)}')
        
        self.nav2_ready = False
        return False
        
    def goal_callback(self, goal_msg):
        """Called when a new goal pose is received from RViz"""
        if not self.nav2_ready:
            self.get_logger().warn('Navigation stack not ready. Goal ignored.')
            return
        
        self.get_logger().info(f'Received goal: x={goal_msg.pose.position.x:.2f}, y={goal_msg.pose.position.y:.2f}')
        
        # If we're already processing a goal, cancel it
        if self.awaiting_result and hasattr(self, 'goal_handle'):
            self.get_logger().info('Canceling previous goal to process new goal')
            self.goal_handle.cancel_goal_async()
            self.awaiting_result = False
        
        # Create waypoint goal with single pose
        goal = FollowWaypoints.Goal()
        goal.poses = [goal_msg]
        
        self.get_logger().info('Sending goal to waypoint follower')
        self.awaiting_result = True
        
        # Send goal to waypoint follower
        send_goal_future = self.action_client.send_goal_async(
            goal, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Called when the goal has been accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            self.awaiting_result = False
            return
            
        self.get_logger().info('Goal accepted by the action server')
        self.goal_handle = goal_handle
        
        # Request for final result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Called periodically while the goal is being executed"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Currently executing waypoint: {feedback.current_waypoint}')
    
    def get_result_callback(self, future):
        """Called when the goal has completed"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Successfully reached the goal position!')
        else:
            self.get_logger().error(f'Failed to reach goal with status: {status}')
            if hasattr(result, 'missed_waypoints') and result.missed_waypoints:
                self.get_logger().error(f'Missed waypoints: {result.missed_waypoints}')
        
        self.awaiting_result = False

def main(args=None):
    rclpy.init(args=args)
    node = GoalListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()