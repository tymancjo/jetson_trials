import serial
import time
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# ser.baudrate = 9600

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    # parity=serial.PARITY_ODD,
    # stopbits=serial.STOPBITS_TWO,
    # bytesize=serial.EIGHTBITS
)

print(ser.name)         # check which port was really used
time.sleep(2)
print("sending...")
msg = f'<1,2,0,0>'
ser.write(msg.encode())
time.sleep(1/100)
msg = f'<41,7,30,0>'
ser.write(msg.encode())

# while (True):
time.sleep(2)
response = str(ser.readline())
print(response)
response = str(ser.readline())
print(response)
ser.close()             # close port
