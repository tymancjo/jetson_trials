# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import jetson.inference
import jetson.utils

import argparse
import sys

import serial
import time
import math

# definitions

def setHandIK(x, y, z=0, h=0, a0=60):
    """
    Inverse Kinematic model for hands
    x,y,z palm position in relative space (-1,-1) to (1,1)
    h - hand number (0-left, 1-right, 3-both)
    the z - controls the gamma only (turn of the shoulder)
    """
    m = math.sqrt(x**2 + y**2)

    if m > 1:
        fi = 180
    else:
        fi = math.degrees(math.acos((m**2 -0.25 -0.25)/(-0.5)))
    
    dzetta = math.degrees(math.atan2(-y,-x)) 
    # gamma = math.degrees(math.atan2(z+0.01,0.1*x))
    gamma = 160 * z 

    if dzetta < 0: dzetta = 360 + dzetta
    if gamma < 0: gamma = 360 + gamma
    gamma *= 160/76 #to map the move to the servo 76deg to 160deg servo
    gamma = max(0,min(160,gamma))

    psi = dzetta - (180 - fi)/2
    psi -= a0
    psi = max(5,min(175,psi))
    fi = max(5,min(175,fi))

    cmd_str = '' 

    if h==0 or h==3:
        cmd_str +=  f'<41,0,{int(max(0,180-(psi)))},0> <41,1,{int(fi)},0> '
        cmd_str += f'<41,2,{int(max(0,160-gamma))},0> '

    if h==1 or h==3:
        cmd_str += f'<41,15,{int(max(0,(psi)))},0> <41,14,{int(180-fi)},0> '
        cmd_str += f'<41,13,{int(max(0,gamma))},0> '
    
    return cmd_str

# connection to serial
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)
print(f'Connected to: {ser.name}') # check which port was really used
time.sleep(1)

print("Loading ML...")

# parse the command line
parser = argparse.ArgumentParser(description="Run pose estimation DNN on a video/image stream.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.poseNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="resnet18-body", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="links,keypoints", help="pose overlay flags (e.g. --overlay=links,keypoints)\nvalid combinations are:  'links', 'keypoints', 'boxes', 'none'")
parser.add_argument("--threshold", type=float, default=0.15, help="minimum detection threshold to use") 
parser.add_argument("--disp", type=int, default=1, help="state to work with display or without") 

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the pose estimation model
# net = jetson.inference.poseNet(opt.network, sys.argv, opt.threshold)
net = jetson.inference.poseNet("resnet18-body", sys.argv, 0.12)

# create video sources & outputs
# camera = jetson.utils.videoSource("csi://0",["--input-width=640", "--input-height=320"])
camera = jetson.utils.videoSource("csi://0",["--input-width=1280", "--input-height=720"])
output = jetson.utils.videoOutput("display://0")

# preparing control
# few initial values to keep the starting point coherent 
persons = 0

the_x_p = 0
the_y_p = 0

head_x_angle = 80
head_x_angle_p = 80
head_x_angle_s = 80

head_x_flag = 1

head_y_angle = 80
head_y_angle_p = 80
head_y_angle_s = 80

last_cmd_time = time.time()
last_head_time = time.time()
last_send_time = time.time()

hand_R_x = 0
hand_R_y = 0
hand_R_z = 0
hand_L_x = 0
hand_L_y = 0
hand_L_z = 0

shoulders_w = 1
pt_6x = 0
pts_x = []
pts_y = []
for _ in range(20):
    pts_x.append(0)
    pts_y.append(0)

# some stuff to keep track of the all body turning (on wheels)
last_turn_time = time.time()
turn_delay = 1.5 # how long to wait to make a turn
body_angle = 0 # to track the body angle position
body_angle_limit = 80 # the total arc for allowed move

# process frames until the user exits
while True:
    # capture the next image
    img = camera.Capture()

    # perform pose estimation (with overlay)
    poses = net.Process(img, overlay=opt.overlay)

    if opt.disp:
        # if the display is suppose to be shown
        # render the image
        output.Render(img)
        # update the title bar
        output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

    # print out performance info
    # net.PrintProfilerTimes()

    ######### Robot control
    # persons is used to determine the behavior 
    # it's time domain filtered to get immune for sudden changes.
    persons = 0.8 * persons + 0.2 * len(poses)
    if persons < 0.1:
        persons = 0
    
    # memorizing this loop execution moment
    now = time.time()

    if len(poses):
        # if any pose (person) is detected
        pose = poses[0]
        points = pose.Keypoints
        last_cmd_time = now

        the_x = 0;
        the_y = 0;
        points_n = 0;

        for p in points:
            # print(p)
            if p.ID in [0,1,2]:
                the_x += p.x
                the_y += p.y
                points_n += 1
            pts_x[p.ID] = p.x
            pts_y[p.ID] = p.y

        if points_n > 0:
            the_x = (the_x / points_n) / img.width
            the_y = 1 - (the_y / points_n) / img.height
            the_x_p = the_x
            the_y_p = the_y
        else:
            the_x = the_x_p
            the_y = the_y_p
        
        print(f'Eyes x:{the_x} y:{the_y}')

        # Handling the  head movements
        head_x_angle = 42 + 90 * (1 - the_x)
        head_x_angle = 0.7 * head_x_angle_p + 0.3 * head_x_angle
        head_x_angle = max(5,min(head_x_angle,175))
        
        head_y_angle = 200 * (the_y - 0.5) 
        head_y_angle = 0.7 * head_y_angle_p + 0.3 * head_y_angle
        head_y_angle = max(5,min(head_y_angle,100))

        print(f'x: {head_x_angle} y: {head_y_angle} body:{body_angle}')
        
        msg = ''
        if now > last_head_time + 200/1000:
            last_head_time = now
            if abs(head_x_angle - head_x_angle_s) > 1:
                msg += f'<41,7,{int(head_x_angle)},0>'
                head_x_angle_s = head_x_angle

            if abs(head_y_angle - head_y_angle_s) > 1:
                msg += f' <41,6,{int(head_y_angle)},0>'
                head_y_angle_s = head_y_angle

            if msg != '':
                print(msg)
                # sending the message to robot via serial
                ser.write(msg.encode())

        head_x_angle_p = head_x_angle
        head_y_angle_p = head_y_angle

        # handling the body movement
        if now > last_turn_time + turn_delay:
            # calculating the head angle vs the body axis
            head_angle_abs = head_x_angle - 85
            if abs(head_angle_abs) > 15:
                # if the head is more than the 30 deg - we will move body
                # making the message to the wheels:
                print(f"Potential move...{head_angle_abs}")
                if -body_angle_limit < body_angle + 0.5*head_angle_abs < body_angle_limit:

                    wheels_msg = f'<1,0,{int(0.5*head_angle_abs)},0>'
                    # sending the message to robot via serial
                    ser.write(wheels_msg.encode())

                    body_angle += head_angle_abs
                    print(f"Body move by {head_angle_abs}")
                else:
                    #limiting the amount of body turn
                    ...
            else:
                # otherwise we just reset the body turn timer. 
                ...
            last_turn_time = now
            ...
    
    # Handling the head if no one in frame
    if persons < 0.5:
        if now > last_cmd_time + 5:
            #if we don't see anyone for 5 sec
            # and we will look left <-> right - like looking around
            last_cmd_time = now

            head_x_angle = 85 - 45 * head_x_flag
            head_x_angle_p = head_x_angle
            head_x_flag *= -1

            msg = f'<41,7,{int(head_x_angle)},0>'
            # sending the message to robot via serial
            ser.write(msg.encode())

    # Handling the hands movement
    if 0.5 < persons < 1.7:
        # if we see single person - we mimic the moves
        f_k = 0.5
        shoulders_w = 1 + math.sqrt( (pts_x[6]-pts_x[5])**2 + (pts_y[6]-pts_y[5])**2 )
        hand_R_x = f_k*hand_R_x + (1-f_k) * max(0, pts_x[6] - pts_x[10])/shoulders_w
        hand_L_x = f_k*hand_L_x + (1-f_k) * max(0, pts_x[9] - pts_x[5])/shoulders_w

        hand_R_y = f_k*hand_R_y + (1-f_k)*(pts_y[10] - pts_y[6])/shoulders_w
        hand_L_y = f_k*hand_L_y + (1-f_k)*(pts_y[9] - pts_y[5])/shoulders_w

        hand_R_z = f_k*hand_R_z + (1-f_k)*(pts_x[6] - pts_x[8])/shoulders_w
        hand_L_z = f_k*hand_L_z + (1-f_k)*(pts_x[7] - pts_x[5])/shoulders_w
    
    elif persons < 0.5:
        # if no one in the frame - we make a gesture
        hand_R_x = 0.5
        hand_L_x = 0.5

        hand_R_y = 0.5
        hand_L_y = 0.5

        hand_R_z = 1
        hand_L_z = 1
    
    else:
        # many persons in the frame - other gesture
        hand_R_x = 0.1
        hand_L_x = 0.1

        hand_R_y = 1 
        hand_L_y = 1

        hand_R_z = 2
        hand_L_z = 2

    # concole info out
    print(f'************** PERSONS {persons} ***********')
    print(f'Hands L x:{hand_L_x}  R x:{hand_R_x} R z:{hand_R_z}')
    print(f'Hands L y:{hand_L_y}  R y:{hand_R_y} R z:{hand_R_z}')

    if now > last_send_time + 50/1000:
        hand_msg = setHandIK(-0.1+hand_L_x, -1.0*hand_L_y, max(0,2*hand_L_z-0.7), h=1)
        # sending the message to robot via serial
        ser.write(hand_msg.encode())
        
        hand_msg = setHandIK(-0.1+hand_R_x, -1.0*hand_R_y, max(0,2*hand_R_z-0.7), h=0)
        # sending the message to robot via serial
        ser.write(hand_msg.encode())

        last_send_time = now

    # exit on input/output EOS
    if not camera.IsStreaming() or not output.IsStreaming():
        break

