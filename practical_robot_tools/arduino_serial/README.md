# Arduino Serial Module

The `arduino_serial` module facilitates communication between a ROS 2 node and an Arduino device over a serial connection. It includes tools for sending and receiving data, enabling seamless integration between ROS 2 and Arduino-based hardware.

## Features

- **ROS 2 Node (`arduino_node.py`)**:

  - Subscribes to the `serial_data` topic to receive commands.
  - Sends commands to the Arduino via a serial connection.
  - Reads responses from the Arduino and logs them.
  - Supports non-blocking keyboard input for manual control.

- **Arduino Example (`arduino_example.ino`)**:
  - Demonstrates how to handle serial communication on the Arduino side.
  - Controls an LED based on received commands (`0`, `1`, or `2`).

## File Structure

- `arduino_serial/arduino_node.py`: ROS 2 node for serial communication.
- `arduino_serial/arduino_example.ino`: Arduino sketch for handling serial commands.
- `test/`: Contains unit tests for code quality and compliance.

## Prerequisites

- **Hardware**:

  - Arduino board (e.g., Uno, Mega, etc.).
  - USB cable for connecting the Arduino to your computer.

- **Software**:
  - ROS 2 installed on your system.
  - Python 3 with required dependencies (`rclpy`, `serial`).
  - Arduino IDE for uploading the sketch to the Arduino.

## Setup

1. **Arduino Setup**:

   - Open `arduino_example.ino` in the Arduino IDE.
   - Connect your Arduino board to your computer.
   - Select the correct board and port in the Arduino IDE.
   - Upload the sketch to the Arduino.

2. **ROS 2 Node Setup**:
   - Ensure the `arduino_serial` package is installed in your ROS 2 workspace.
   - Build the workspace:
     ```bash
     colcon build
     ```
   - Source the workspace:
     ```bash
     source install/setup.bash
     ```

## Usage

1. **Flash the Arduino**:

   - Open the `arduino_example.ino` file in the Arduino IDE.
   - Connect your Arduino board to your computer via USB.
   - Select the correct board and port in the Arduino IDE.
   - Click the "Upload" button to flash the sketch to the Arduino.
   - Once uploaded, the Arduino will be ready to receive commands to control the built-in LED.

2. **Run the ROS 2 Node**:

   - Start the ROS 2 node to enable communication with the Arduino:
     ```bash
     ros2 run arduino_serial arduino_node
     ```

3. **Send Commands**:

   - Use the ROS 2 topic `/serial_data` to send commands to the Arduino:
     ```bash
     ros2 topic pub /serial_data std_msgs/String "data: '1'"
     ```
     - `0`: Turns off the built-in LED.
     - `1`: Turns on the built-in LED.
     - `2`: Logs a message on the Arduino without affecting the LED.
     - Any other input will result in an "Invalid input" message.

4. **Keyboard Input (Optional)**:

   - While the ROS 2 node is running, you can also send commands directly from the terminal using the keyboard:
     - Enter `0`, `1`, or `2` to send commands to the Arduino.
     - Enter `q` to quit the keyboard input loop.

5. **Observe the Arduino**:
   - The built-in LED on the Arduino will blink based on the commands received:
     - `0`: LED turns off.
     - `1`: LED turns on.
     - `2`: No LED action, but a message is logged in the Arduino's serial monitor.
   - The Arduino will also send feedback messages (e.g., "LED ON", "LED OFF") that are logged by the ROS 2 node.
