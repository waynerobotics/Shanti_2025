import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3
import serial
from math import sin, cos

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Define wheel_base (distance between wheels in meters)
        self.wheel_base = 0.3  # Adjust to your robot's actual wheel base

    def parse_serial_data(self, line):
        data = {}
        self.get_logger().debug(f"Received serial data: {line}")
        for pair in line.split(','):
            try:
                if ':' in pair:
                    key, value = pair.split(':', 1)
                    data[key] = value
                else:
                    self.get_logger().warning(f"Skipping malformed data segment: '{pair}'")
            except Exception as e:
                self.get_logger().error(f"Error parsing data segment '{pair}': {e}")
        
        # Check if required data is available
        required_keys = ['E1_RPM', 'E1_DIR', 'E2_RPM', 'E2_DIR']
        for key in required_keys:
            if key not in data:
                self.get_logger().error(f"Missing required data: {key}")
                # Provide default values if missing
                if key.endswith('_RPM'):
                    data[key] = '0'
                elif key.endswith('_DIR'):
                    data[key] = 'F'  # Default to forward
        
        return data

    def update_odometry(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            data = self.parse_serial_data(line)
            
            # Extract RPM and direction for both wheels
            rpm1 = float(data['E1_RPM'])
            dir1 = 1 if data['E1_DIR'] == 'F' else -1
            rpm2 = float(data['E2_RPM'])
            dir2 = 1 if data['E2_DIR'] == 'F' else -1
            
            # Compute wheel velocities (m/s)
            wheel_radius = 0.1016
            v_left = dir1 * rpm1 * (2 * 3.1416 * wheel_radius) / 60
            v_right = dir2 * rpm2 * (2 * 3.1416 * wheel_radius) / 60
            
            # Differential drive kinematics
            linear_vel = (v_left + v_right) / 2
            angular_vel = (v_right - v_left) / (2 * self.wheel_base)
            
            current_time = self.get_clock().now()
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            
            # Update pose (simplified integration)
            self.x += linear_vel * dt * cos(self.theta)
            self.y += linear_vel * dt * sin(self.theta)
            self.theta += angular_vel * dt
            
            # Publish Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Create proper pose message
            pose = Pose()
            pose.position.x = self.x
            pose.position.y = self.y
            pose.position.z = 0.0
            
            # Create quaternion for rotation
            quat = Quaternion()
            quat.w = cos(self.theta/2)
            quat.x = 0.0
            quat.y = 0.0
            quat.z = sin(self.theta/2)
            pose.orientation = quat
            odom_msg.pose.pose = pose
            
            # Create twist message
            twist = Twist()
            twist.linear.x = linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular_vel
            odom_msg.twist.twist = twist
            
            self.odom_pub.publish(odom_msg)
            self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    
    try:
        # Create a timer to regularly update odometry
        node.create_timer(0.1, node.update_odometry)  # 10Hz update rate
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()