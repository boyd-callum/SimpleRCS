import serial
import time

# Configure the serial port
serial_port = '/dev/cu.usbserial-140'  # 
baud_rate = 9600  # Match the baud rate with the arduino
output_file = 'throttledata1.csv'

# Open the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)
time.sleep(2)  # Give some time for the serial connection to establish

line_count = 0
wanted_line_count = 500  # Number of lines to read

# Open the text file
with open(output_file, mode='w') as file:
    while line_count < wanted_line_count:
        line = ser.readline().decode('utf-8').strip()  # Read a line of serial data
        if line:
            file.write(line + '\n')
            print(f"linecount: {line_count}, {line}")  # Print to console (optional)
            line_count += 1
    ser.close()
        
