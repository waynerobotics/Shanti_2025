#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener


class CustomLifecycleManager(Node):
    """
    Custom lifecycle manager that monitors specific conditions to trigger
    lifecycle state transitions for UTM map transform publisher and GPS map transformer.
    
    It activates the UTM map transform publisher when map odometry is available,
    and activates the GPS map transformer when the UTM->map transform is available.
    """

    def __init__(self):
        super().__init__('custom_lifecycle_manager')
        
        # Parameters
        self.declare_parameter('map_odom_topic', '/odometry/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('transform_check_period', 1.0)
        self.declare_parameter('utm_map_transform_publisher_node', 'utm_map_transform_publisher')
        self.declare_parameter('gps_map_transformer_node', 'gps_map_transformer')
        
        # Get parameters
        self.map_odom_topic = self.get_parameter('map_odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.transform_check_period = self.get_parameter('transform_check_period').value
        self.utm_map_transform_publisher_node = self.get_parameter('utm_map_transform_publisher_node').value
        self.gps_map_transformer_node = self.get_parameter('gps_map_transformer_node').value
        
        # Initialize state tracking
        self.utm_map_transform_publisher_configured = False
        self.utm_map_transform_publisher_activated = False
        self.gps_map_transformer_configured = False
        self.gps_map_transformer_activated = False
        self.map_odom_received = False
        self.utm_transform_available = False
        
        # Used to track pending service calls
        self.pending_requests = {}
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create ChangeState service clients
        self.utm_map_transform_publisher_state_client = self.create_client(
            ChangeState, f'/{self.utm_map_transform_publisher_node}/change_state')
        self.gps_map_transformer_state_client = self.create_client(
            ChangeState, f'/{self.gps_map_transformer_node}/change_state')
        
        # Create GetState service clients
        self.utm_map_transform_publisher_get_state_client = self.create_client(
            GetState, f'/{self.utm_map_transform_publisher_node}/get_state')
        self.gps_map_transformer_get_state_client = self.create_client(
            GetState, f'/{self.gps_map_transformer_node}/get_state')
        
        # Wait a little for service clients to be available
        self.utm_map_transform_publisher_state_client.wait_for_service(timeout_sec=0.1)
        self.gps_map_transformer_state_client.wait_for_service(timeout_sec=0.1)
        self.utm_map_transform_publisher_get_state_client.wait_for_service(timeout_sec=0.1)
        self.gps_map_transformer_get_state_client.wait_for_service(timeout_sec=0.1)
        
        # Create map odometry subscription
        self.map_odom_sub = self.create_subscription(
            Odometry,
            self.map_odom_topic,
            self.map_odom_callback,
            10
        )
        
        # Create transform check timer
        self.transform_check_timer = self.create_timer(
            self.transform_check_period,
            self.check_transform_availability
        )
        
        # Create lifecycle management timer - slower to avoid excessive service calls
        self.lifecycle_timer = self.create_timer(3.0, self.manage_lifecycle)
        
        self.get_logger().info('Custom Lifecycle Manager initialized')
    
    def map_odom_callback(self, msg):
        """Callback for map odometry messages."""
        if not self.map_odom_received:
            self.map_odom_received = True
            self.get_logger().info('Received first map odometry message')
    
    def check_transform_availability(self):
        """Check if the UTM->map transform is available."""
        try:
            # Check if transform is available by looking it up with a timeout
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)  # Short timeout
            )
            
            if not self.utm_transform_available:
                self.utm_transform_available = True
                self.get_logger().info('UTM->map transform is now available')
        
        except TransformException:
            if self.utm_transform_available:
                self.utm_transform_available = False
                self.get_logger().warn('UTM->map transform is no longer available')
    
    def get_state_callback(self, future, node_name):
        """Callback for GetState service responses."""
        try:
            response = future.result()
            if response is not None:
                state_id = response.current_state.id
                self.get_logger().debug(f'Current state of {node_name}: {self.state_id_to_string(state_id)}')
                return state_id
            else:
                self.get_logger().error(f'Failed to get state for {node_name}: None response')
        except Exception as e:
            self.get_logger().error(f'Error getting state for {node_name}: {str(e)}')
        
        # Remove from pending requests
        if (node_name, 'get_state') in self.pending_requests:
            del self.pending_requests[(node_name, 'get_state')]
        
        return None
    
    def change_state_callback(self, future, node_name, transition_id):
        """Callback for ChangeState service responses."""
        try:
            response = future.result()
            if response is not None and response.success:
                target_state = self.get_target_state(transition_id)
                self.get_logger().info(f'Successfully changed state of {node_name} to {self.state_id_to_string(target_state)}')
                
                # Update our state tracking
                if transition_id == Transition.TRANSITION_CONFIGURE:
                    if node_name == self.utm_map_transform_publisher_node:
                        self.utm_map_transform_publisher_configured = True
                    else:
                        self.gps_map_transformer_configured = True
                elif transition_id == Transition.TRANSITION_ACTIVATE:
                    if node_name == self.utm_map_transform_publisher_node:
                        self.utm_map_transform_publisher_activated = True
                    else:
                        self.gps_map_transformer_activated = True
                elif transition_id == Transition.TRANSITION_DEACTIVATE:
                    if node_name == self.utm_map_transform_publisher_node:
                        self.utm_map_transform_publisher_activated = False
                    else:
                        self.gps_map_transformer_activated = False
            else:
                error_msg = 'None response' if response is None else 'Failed response'
                self.get_logger().error(f'Failed to change state for {node_name}: {error_msg}')
        except Exception as e:
            self.get_logger().error(f'Error changing state for {node_name}: {str(e)}')
        
        # Remove from pending requests
        if (node_name, transition_id) in self.pending_requests:
            del self.pending_requests[(node_name, transition_id)]
    
    def get_target_state(self, transition_id):
        """Get the target state for a transition."""
        if transition_id == Transition.TRANSITION_CONFIGURE:
            return State.PRIMARY_STATE_INACTIVE
        elif transition_id == Transition.TRANSITION_ACTIVATE:
            return State.PRIMARY_STATE_ACTIVE
        elif transition_id == Transition.TRANSITION_DEACTIVATE:
            return State.PRIMARY_STATE_INACTIVE
        elif transition_id == Transition.TRANSITION_CLEANUP:
            return State.PRIMARY_STATE_UNCONFIGURED
        else:
            return None
    
    def state_id_to_string(self, state_id):
        """Convert a state ID to a string representation."""
        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            return "UNCONFIGURED"
        elif state_id == State.PRIMARY_STATE_INACTIVE:
            return "INACTIVE"
        elif state_id == State.PRIMARY_STATE_ACTIVE:
            return "ACTIVE"
        elif state_id == State.PRIMARY_STATE_FINALIZED:
            return "FINALIZED"
        else:
            return f"UNKNOWN ({state_id})"
    
    def get_state(self, node_name):
        """Get the current state of a lifecycle node."""
        # Skip if a get_state request is already pending for this node
        # if (node_name, 'get_state') in self.pending_requests:
        #     self.get_logger().debug(f'Get state request already pending for {node_name}')
        #     return None
        
        # # Determine which client to use
        # if node_name == self.utm_map_transform_publisher_node:
        #     client = self.utm_map_transform_publisher_get_state_client
        # else:
        #     client = self.gps_map_transformer_get_state_client
        
        # # Skip if service not available
        # if not client.service_is_ready():
        #     self.get_logger().debug(f'Get state service not ready for {node_name}')
        #     return None
        
        # # Call the service
        # request = GetState.Request()
        # future = client.call_async(request)
        # self.pending_requests[(node_name, 'get_state')] = future
        # future.add_done_callback(lambda f: self.get_state_callback(f, node_name))
        # return None
        pass
    
    def change_state(self, node_name, transition_id):
        """Send a state change request."""
        # Skip if this exact transition is already pending
        if (node_name, transition_id) in self.pending_requests:
            self.get_logger().debug(f'Transition {transition_id} already pending for {node_name}')
            return
        
        # Determine which client to use
        if node_name == self.utm_map_transform_publisher_node:
            client = self.utm_map_transform_publisher_state_client
        else:
            client = self.gps_map_transformer_state_client
        
        # Skip if service not available
        if not client.service_is_ready():
            self.get_logger().warning(f'Change state service not ready for {node_name}')
            return
        
        # Create the request
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        # Send the request
        self.get_logger().info(f'Requesting transition {self.transition_id_to_string(transition_id)} for {node_name}')
        future = client.call_async(request)
        self.pending_requests[(node_name, transition_id)] = future
        future.add_done_callback(lambda f: self.change_state_callback(f, node_name, transition_id))
    
    def transition_id_to_string(self, transition_id):
        """Convert a transition ID to a string representation."""
        if transition_id == Transition.TRANSITION_CREATE:
            return "CREATE"
        elif transition_id == Transition.TRANSITION_CONFIGURE:
            return "CONFIGURE"
        elif transition_id == Transition.TRANSITION_CLEANUP:
            return "CLEANUP"
        elif transition_id == Transition.TRANSITION_ACTIVATE:
            return "ACTIVATE"
        elif transition_id == Transition.TRANSITION_DEACTIVATE:
            return "DEACTIVATE"
        elif transition_id == Transition.TRANSITION_DESTROY:
            return "DESTROY"
        else:
            return f"UNKNOWN ({transition_id})"
    
    def configure_node(self, node_name):
        """Configure a lifecycle node."""
        self.change_state(node_name, Transition.TRANSITION_CONFIGURE)
    
    def activate_node(self, node_name):
        """Activate a lifecycle node."""
        self.change_state(node_name, Transition.TRANSITION_ACTIVATE)
    
    def deactivate_node(self, node_name):
        """Deactivate a lifecycle node."""
        self.change_state(node_name, Transition.TRANSITION_DEACTIVATE)
    
    def cleanup_node(self, node_name):
        """Clean up a lifecycle node."""
        self.change_state(node_name, Transition.TRANSITION_CLEANUP)

    def manage_lifecycle(self):
        """Manage the lifecycle of the nodes based on conditions."""
        # First, initiate get_state calls to check the current state of each node
        utm_state = self.get_state(self.utm_map_transform_publisher_node)
        gps_state = self.get_state(self.gps_map_transformer_node)
        
        # Handle node lifecycle based on our internal state tracking and conditions
        # Configure nodes if not already configured
        if not self.utm_map_transform_publisher_configured:
            self.get_logger().debug(f'Requesting configuration of {self.utm_map_transform_publisher_node}')
            self.configure_node(self.utm_map_transform_publisher_node)
        
        if not self.gps_map_transformer_configured:
            self.get_logger().debug(f'Requesting configuration of {self.gps_map_transformer_node}')
            self.configure_node(self.gps_map_transformer_node)
        
        # Activate UTM Map Transform Publisher when map odometry is available
        if (self.utm_map_transform_publisher_configured and 
                not self.utm_map_transform_publisher_activated and 
                self.map_odom_received):
            self.get_logger().debug(f'Requesting activation of {self.utm_map_transform_publisher_node}')
            self.activate_node(self.utm_map_transform_publisher_node)
        
        # Activate GPS Map Transformer when the transform is available
        if (self.gps_map_transformer_configured and 
                not self.gps_map_transformer_activated and 
                self.utm_transform_available):
            self.get_logger().debug(f'Requesting activation of {self.gps_map_transformer_node}')
            self.activate_node(self.gps_map_transformer_node)
        
        # Deactivate GPS Map Transformer if transform becomes unavailable
        if (self.gps_map_transformer_activated and 
                not self.utm_transform_available):
            self.get_logger().debug(f'Requesting deactivation of {self.gps_map_transformer_node}')
            self.deactivate_node(self.gps_map_transformer_node)


def main(args=None):
    """Entry point for the Custom Lifecycle Manager node."""
    rclpy.init(args=args)
    
    # Create the node
    node = CustomLifecycleManager()
    
    # Create executor and spin
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()