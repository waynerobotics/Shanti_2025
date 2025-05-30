std_msgs/Header header

# Connection status
bool connected

# Firmware information
string firmware_version 

# Motor 1 data
int32 m1_encoder
int32 m1_speed  # Quadrature counts per second
int32 m1_target_speed
int16 m1_duty  # PWM duty cycle (-32768 to +32767)
int32 m1_current  # Motor current in milliamps

# Motor 2 data
int32 m2_encoder
int32 m2_speed  # Quadrature counts per second
int32 m2_target_speed
int16 m2_duty  # PWM duty cycle (-32768 to +32767)
int32 m2_current  # Motor current in milliamps

# Temperature and voltage information
float32 main_battery_voltage  # Main battery voltage
float32 logic_battery_voltage  # Logic battery voltage
float32 board_temperature  # Board temperature in degrees C

# Error status
uint8 error_status  # Bit-encoded error flags
string error_message  # Human-readable error message
