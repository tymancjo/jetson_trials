import serial
import time
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# ser.baudrate = 9600

ser = serial.Serial(
    port='/dev/cu.usbserial-120',
    baudrate=115200,
    # parity=serial.PARITY_ODD,
    # stopbits=serial.STOPBITS_TWO,
    # bytesize=serial.EIGHTBITS
)

print(ser.name)         # check which port was really used
time.sleep(2)
# print("sending...")
# time.sleep(1/100)
# msg = f'<41,15,80> <41,8,80>'
# ser.write(msg.encode())

# while (True):
list = """
<41,15,100> <41,8,30>;
<41,15,125> <41,8,55>;
<41,15,125> <41,8,115>;
<41,15,125> <41,8,125>;
<41,15,90> <41,8,110>;
<41,15,70> <41,8,100>;
<41,15,55> <41,8,80>;
<41,15,50> <41,8,40>;
"""

step_list = [
    (100,30),
    (125,55),
    (125,115),
    (125,125),
    (90,110),
    (70,100),
    (55,80),
    (50,40)
]


# zerowanie serwo na 90 stopni

A = 90
while True:
    zeromsg = "<41"
    for k in range(8):
        zeromsg += f",{A}"
    zeromsg += ">"
    ser.write(zeromsg.encode())
    time.sleep(1.525)
    if A==90: 
        A = 0
    else:
        A = 90

# print(zeromsg)
# ser.write(zeromsg.encode())
# time.sleep(1)

# print(execute)
# msg = "<41,15,90> <41,8,90>"
# ser.write(msg.encode())

# while True:
#     for _ in range (2):
#         for msg in execute[::-1]:
#             time.sleep(0.3)
#             ser.write(msg.encode())

#     for _ in range (2):
#         for msg in execute[::1]:
#             time.sleep(0.3)
#             ser.write(msg.encode())



time.sleep(2)
response = str(ser.readline())
print(response)
response = str(ser.readline())
print(response)
ser.close()             # close port
