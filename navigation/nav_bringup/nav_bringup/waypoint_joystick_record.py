import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
import yaml
import os
from datetime import datetime

class JoystickGPSLogger(Node):
    def __init__(self):
        super().__init__('joystick_gps_logger')
        
        # Declare parameters
        self.declare_parameter('output_directory', '~/ros2_ws/src/Shanti_2025/navigation/nav_bringup/params/')
        self.declare_parameter('joystick_button_index', 0)  # Button index for logging waypoints

        # Get parameters
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        self.joystick_button_index = self.get_parameter('joystick_button_index').get_parameter_value().integer_value

        # Initialize variables
        self.gps_data = None
        self.joystick_pressed = False

        # Create a unique filename for the session
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = os.path.join(self.output_directory, f'gps_waypoints_session_{timestamp}.yaml')

        # Subscribers
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_callback, 10)

        self.get_logger().info(f'Joystick GPS Logger Node has been started. Logging to {self.filename}')

    def joy_callback(self, msg):
        # Check if the specified button is pressed
        if msg.buttons[self.joystick_button_index] == 1 and not self.joystick_pressed:
            self.joystick_pressed = True
            self.log_gps_coordinates()
            self.get_logger().info('Button pressed, logging GPS coordinates.')
        elif msg.buttons[self.joystick_button_index] == 0:
            self.joystick_pressed = False

    def gps_callback(self, msg):
        # Store the latest GPS data
        self.gps_data = msg

    def log_gps_coordinates(self):
        if self.gps_data is None:
            self.get_logger().warn('No GPS data available to log.')
            return

        # Prepare the GPS data as a [latitude, longitude] pair
        gps_entry = [self.gps_data.latitude, self.gps_data.longitude]

        # Append the GPS data to the YAML file
        try:
            if not os.path.exists(self.filename):
                # Create a new file with a list structure
                with open(self.filename, 'w') as file:
                    yaml.dump({'waypoints': [gps_entry]}, file, default_flow_style=False)
            else:
                # Append to the existing file
                with open(self.filename, 'r') as file:
                    data = yaml.safe_load(file) or {'waypoints': []}
                data['waypoints'].append(gps_entry)
                with open(self.filename, 'w') as file:
                    yaml.dump(data, file, default_flow_style=False)

            self.get_logger().info(f'GPS coordinates logged: {gps_entry}')
        except Exception as e:
            self.get_logger().error(f'Failed to write GPS coordinates to file: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickGPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()