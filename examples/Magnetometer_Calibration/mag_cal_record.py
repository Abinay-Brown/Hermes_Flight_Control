
import serial

# Configure the serial port
serial_port = '/dev/ttyACM0'  # Change this to your serial port
baud_rate = 9600  # Change this to match your device's baud rate

# Open serial port
ser = serial.Serial(serial_port, baud_rate)

# Open a file for writing
output_file = 'mag_cal.txt'
with open(output_file, 'w') as file:
    try:
        while True:
            # Read line from serial port
            line = ser.readline().decode().strip()

            # Write the line to the text file
            file.write(line + '\n')

            # Print the line to console (optional)
            print(line)

    except KeyboardInterrupt:
        print("Keyboard Interrupt. Exiting.")
        ser.close()
