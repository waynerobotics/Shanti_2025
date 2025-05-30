#!/usr/bin/env python3
"""
RoboClaw Keyboard Test (Simple Version)

This is a simplified version of the keyboard test script that doesn't use the ReadVersion method.
It allows direct control of both motors on a single RoboClaw controller without using ROS2.

Controls:
- W/S: Increase/decrease motor 1 speed
- E/D: Increase/decrease motor 2 speed
- Q/A: Increase/decrease both motors equally
- Space: Stop all motors
- 0: Reset to zero
- ESC: Exit

Requirements:
- Python 3
- curses (standard library)
- roboclaw_3 library
"""

import curses
import sys
import time
import signal

# Try to import the RoboClaw driver
try:
    from differential_drive_base.roboclaw_3 import Roboclaw
except ImportError as e:
    try:
        # Alternative import location
        from roboclaw_3 import Roboclaw
    except ImportError:
        print(f"Error importing Roboclaw library: {e}")
        print("Make sure the roboclaw_3 library is installed correctly")
        sys.exit(1)

# Configuration (edit these values as needed)
DEFAULT_PORT = '/dev/ttyACM0'  # Default serial port
DEFAULT_BAUDRATE = 38400       # Default baud rate
DEFAULT_ADDRESS = 0x80         # Default controller address
SPEED_INCREMENT = 2000         # Speed increment per key press
MAX_SPEED = 32767              # Maximum allowed speed value

class RoboclawKeyboardControl:
    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE, address=DEFAULT_ADDRESS):
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.roboclaw = None
        self.m1_speed = 0
        self.m2_speed = 0
        self.screen = None
        
    def connect(self):
        """Connect to the RoboClaw controller"""
        try:
            self.roboclaw = Roboclaw(self.port, self.baudrate)
            success = self.roboclaw.Open()
            if success:
                # Skip version check - just assume connection is good if port opens
                return True, f"Connected to RoboClaw on {self.port}"
            else:
                return False, f"Failed to open port {self.port}"
        except Exception as e:
            return False, f"Error connecting to RoboClaw: {e} (Type: {type(e).__name__})"
    
    def set_motor_speeds(self):
        """Set the motor speeds based on current values"""
        try:
            if self.roboclaw:
                self.roboclaw.DutyM1(self.address, self.m1_speed)
                self.roboclaw.DutyM2(self.address, self.m2_speed)
                return True
        except Exception as e:
            self.screen.addstr(10, 0, f"Error setting speeds: {e}" + " " * 20)
            return False
    
    def stop_motors(self):
        """Stop both motors"""
        self.m1_speed = 0
        self.m2_speed = 0
        if self.roboclaw:
            try:
                self.roboclaw.DutyM1(self.address, 0)
                self.roboclaw.DutyM2(self.address, 0)
            except Exception as e:
                if self.screen:
                    self.screen.addstr(10, 0, f"Error stopping motors: {e}" + " " * 20)
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_motors()
        if self.roboclaw:
            # Close the serial port
            try:
                self.roboclaw._port.close()
            except:
                pass
    
    def run(self):
        """Run the keyboard control interface"""
        # Initialize curses
        self.screen = curses.initscr()
        curses.noecho()  # Don't echo key presses
        curses.cbreak()  # React to keys without Enter
        self.screen.keypad(True)  # Enable special keys
        
        try:
            # Connect to the RoboClaw
            success, message = self.connect()
            self.screen.addstr(0, 0, message)
            if not success:
                self.screen.addstr(1, 0, "Press any key to exit...")
                self.screen.refresh()
                self.screen.getch()
                return
            
            # Print debug info
            self.screen.addstr(1, 0, f"Port: {self.port}, Baud: {self.baudrate}, Address: {self.address:#x}")
            
            # Display controls
            self.screen.addstr(2, 0, "Controls:")
            self.screen.addstr(3, 0, "W/S: Increase/decrease motor 1 speed")
            self.screen.addstr(4, 0, "E/D: Increase/decrease motor 2 speed")
            self.screen.addstr(5, 0, "Q/A: Increase/decrease both motors")
            self.screen.addstr(6, 0, "Space: Stop all motors")
            self.screen.addstr(7, 0, "0: Reset to zero")
            self.screen.addstr(8, 0, "ESC: Exit")
            
            # Main loop
            while True:
                # Display current speed values
                self.screen.addstr(12, 0, f"Motor 1 Speed: {self.m1_speed:7d}")
                self.screen.addstr(13, 0, f"Motor 2 Speed: {self.m2_speed:7d}")
                self.screen.refresh()
                
                # Get key input (non-blocking)
                self.screen.timeout(100)  # 100ms timeout for getch()
                key = self.screen.getch()
                
                if key == -1:  # No key pressed
                    continue
                elif key == 27:  # ESC
                    break
                elif key == ord(' '):  # Space
                    self.stop_motors()
                elif key == ord('0'):  # 0 key
                    self.m1_speed = 0
                    self.m2_speed = 0
                    self.set_motor_speeds()
                
                # Handle motor speed changes with error protection
                try:
                    if key == ord('w') or key == ord('W'):  # W - Increase motor 1
                        self.m1_speed = min(self.m1_speed + SPEED_INCREMENT, MAX_SPEED)
                        self.set_motor_speeds()
                    elif key == ord('s') or key == ord('S'):  # S - Decrease motor 1
                        self.m1_speed = max(self.m1_speed - SPEED_INCREMENT, -MAX_SPEED)
                        self.set_motor_speeds()
                    elif key == ord('e') or key == ord('E'):  # E - Increase motor 2
                        self.m2_speed = min(self.m2_speed + SPEED_INCREMENT, MAX_SPEED)
                        self.set_motor_speeds()
                    elif key == ord('d') or key == ord('D'):  # D - Decrease motor 2
                        self.m2_speed = max(self.m2_speed - SPEED_INCREMENT, -MAX_SPEED)
                        self.set_motor_speeds()
                    elif key == ord('q') or key == ord('Q'):  # Q - Increase both
                        self.m1_speed = min(self.m1_speed + SPEED_INCREMENT, MAX_SPEED)
                        self.m2_speed = min(self.m2_speed + SPEED_INCREMENT, MAX_SPEED)
                        self.set_motor_speeds()
                    elif key == ord('a') or key == ord('A'):  # A - Decrease both
                        self.m1_speed = max(self.m1_speed - SPEED_INCREMENT, -MAX_SPEED)
                        self.m2_speed = max(self.m2_speed - SPEED_INCREMENT, -MAX_SPEED)
                        self.set_motor_speeds()
                except Exception as e:
                    self.screen.addstr(15, 0, f"Error during key handling: {e}" + " " * 30)
                    self.screen.refresh()
                
        except Exception as e:
            self.screen.addstr(15, 0, f"Error: {e}")
            self.screen.refresh()
            self.screen.getch()
        finally:
            # Clean up
            self.cleanup()
            
            # Restore terminal
            curses.nocbreak()
            self.screen.keypad(False)
            curses.echo()
            curses.endwin()

def signal_handler(sig, frame):
    """Handle Ctrl+C and other signals"""
    print("\nExiting...")
    sys.exit(0)

def main():
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='RoboClaw Keyboard Control (Simple Version)')
    parser.add_argument('-p', '--port', default=DEFAULT_PORT, help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('-b', '--baudrate', type=int, default=DEFAULT_BAUDRATE, help=f'Baud rate (default: {DEFAULT_BAUDRATE})')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=DEFAULT_ADDRESS, 
                        help=f'RoboClaw address in hex (default: {DEFAULT_ADDRESS:#x})')
    args = parser.parse_args()
    
    # Start the controller
    controller = RoboclawKeyboardControl(args.port, args.baudrate, args.address)
    controller.run()

if __name__ == "__main__":
    main()
