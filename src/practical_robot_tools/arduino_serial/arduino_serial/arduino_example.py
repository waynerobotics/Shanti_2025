import serial
import time

# Set up the serial connection (adjust '/dev/ttyUSB0' and baudrate as needed)
ser = serial.Serial('COM5', 9600, timeout=1)
# ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for the connection to initialize

# Function to send data to Arduino
def send_data(data):
    ser.write(f'{data}\n'.encode())

# Main loop to take user input and send to Arduino
try:
    while True:
        user_input = input("Enter a number (0, 1, or 2) to send to Arduino: ")
        if user_input in ['0', '1', '2']:
            send_data(user_input)
        else:
            print("Invalid input. Please enter 0, 1, or 2.")
except KeyboardInterrupt:
    print("Exiting...")

# Close the serial connection
ser.close()

