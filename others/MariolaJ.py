import jetson.inference
import jetson.utils
import serial
import time

# connection to serial
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
)
print(f'Connected to: {ser.name}') # check which port was really used
time.sleep(1)

print("Loading ML...")
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = jetson.utils.videoSource("csi://0",["--input-width=640", "--input-height=320"])      # '/dev/video0' for V4L2
display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file

error_x = 0
error_y = 0
head_x_angle = 80
head_x_angle_p = 80
head_x_angle_s = 80

head_y_angle = 80
head_y_angle_p = 80
head_y_angle_s = 80

last_cmd_time = time.time()
last_send_time = time.time()

while display.IsStreaming():
        img = camera.Capture()
        screen_area =  img.width * img.height
        detections = net.Detect(img)

        people_x = []
        people_y = []
        people_a = []

        for d in detections:
            if d.ClassID == 1:
                x = d.Center[0] / img.width
                y = 1 - d.Center[1] / img.height
                a = d.Area / screen_area

                # print(f'x:{x} y:{y} a:{a}');
                people_x.append(x)
                people_y.append(y)
                people_a.append(a)
            
        if len(people_a):
            detect_x = sum(people_x) / len(people_x)
            detect_y = sum(people_y) / len(people_y)
            detect_a = sum(people_a) / len(people_a)

            error_x = 0.5 - detect_x
            error_y = 0.5 - detect_y


            now = time.time()
            # if now > last_cmd_time + 5/1000:
            if True:

                head_x_angle = 5 + 160 * (1 - detect_x)
                head_x_angle = 0.9 * head_x_angle_p + 0.1 * head_x_angle
                head_x_angle = max(5,min(head_x_angle,175))
                
                head_y_angle = 120 * (detect_y - 0.1) 
                head_y_angle = 0.9 * head_y_angle_p + 0.1 * head_y_angle
                head_y_angle = max(5,min(head_y_angle,100))

                # print(f'x: {head_x_angle} y: {head_y_angle}')
                
                if now > last_send_time + 1/1000:
                    msg = ''
                    if abs(head_x_angle - head_x_angle_s) > 1:
                        msg += f'<41,7,{int(head_x_angle)},0>'
                        head_x_angle_s = head_x_angle

                    if abs(head_y_angle - head_y_angle_s) > 1:
                        msg += f' <41,6,{int(head_y_angle)},0>'
                        head_y_angle_s = head_y_angle

                    if msg != '':
                        print(msg)
                        ser.write(msg.encode())

                    last_send_time = now

                head_x_angle_p = head_x_angle
                head_y_angle_p = head_y_angle

                last_cmd_time = now


        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
