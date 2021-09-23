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
    x,y palm position in relative space (-1,-1) to (1,1)
    h - hand number
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
    print(f'gamma: {gamma}')

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

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the pose estimation model
# net = jetson.inference.poseNet(opt.network, sys.argv, opt.threshold)
net = jetson.inference.poseNet("resnet18-body", sys.argv, 0.17)

# create video sources & outputs
# camera = jetson.utils.videoSource("csi://0",["--input-width=640", "--input-height=320"])
camera = jetson.utils.videoSource("csi://0",["--input-width=1280", "--input-height=720"])
# camera = jetson.utils.videoSource("csi://0",["--input-width=1280", "--input-height=780"])
output = jetson.utils.videoOutput("display://0")

# preparing head control
the_x_p = 0
the_y_p = 0

head_x_angle = 80
head_x_angle_p = 80
head_x_angle_s = 80

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


# process frames until the user exits
while True:
    # capture the next image
    img = camera.Capture()

    # perform pose estimation (with overlay)
    poses = net.Process(img, overlay=opt.overlay)

    # print the pose results
    print("detected {:d} objects in image".format(len(poses)))

    # for pose in poses:
    #     print(pose)
    #     print(pose.Keypoints)
    #     print('Links', pose.Links)

    # render the image
    output.Render(img)

    # update the title bar
    output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

    # print out performance info
    net.PrintProfilerTimes()

    ######### Robot control
    if len(poses):

        pose = poses[0]
        points = pose.Keypoints

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
        
        f_k = 0.1

        shoulders_w = 1 + math.sqrt( (pts_x[6]-pts_x[5])**2 + (pts_y[6]-pts_y[5])**2 )
        hand_R_x = f_k*hand_R_x + (1-f_k) * max(0, pts_x[6] - pts_x[10])/shoulders_w
        hand_L_x = f_k*hand_L_x + (1-f_k) * max(0, pts_x[9] - pts_x[5])/shoulders_w

        hand_R_y = f_k*hand_R_y + (1-f_k)*(pts_y[10] - pts_y[6])/shoulders_w
        hand_L_y = f_k*hand_L_y + (1-f_k)*(pts_y[9] - pts_y[5])/shoulders_w

        hand_R_z = f_k*hand_R_z + (1-f_k)*(pts_x[6] - pts_x[8])/shoulders_w
        hand_L_z = f_k*hand_L_z + (1-f_k)*(pts_x[7] - pts_x[5])/shoulders_w

        print(f'Hands L x:{hand_L_x}  R x:{hand_R_x} R z:{hand_R_z}')
        print(f'Hands L y:{hand_L_y}  R y:{hand_R_y} R z:{hand_R_z}')


        now = time.time()
        if True:
            head_x_angle = 42 + 90 * (1 - the_x)
            head_x_angle = 0.7 * head_x_angle_p + 0.3 * head_x_angle
            head_x_angle = max(5,min(head_x_angle,175))
            
            head_y_angle = 200 * (the_y - 0.5) 
            head_y_angle = 0.7 * head_y_angle_p + 0.3 * head_y_angle
            head_y_angle = max(5,min(head_y_angle,100))

            print(f'x: {head_x_angle} y: {head_y_angle}')
            
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
                    ser.write(msg.encode())

            if now > last_send_time + 50/1000:
                hand_msg = setHandIK(-0.1+hand_L_x, -1.0*hand_L_y, max(0,2*hand_L_z-0.7), h=1)
                ser.write(hand_msg.encode())
                
                hand_msg = setHandIK(-0.1+hand_R_x, -1.0*hand_R_y, max(0,2*hand_R_z-0.7), h=0)
                ser.write(hand_msg.encode())

                last_send_time = now

            head_x_angle_p = head_x_angle
            head_y_angle_p = head_y_angle

            last_cmd_time = now

    # exit on input/output EOS
    if not camera.IsStreaming() or not output.IsStreaming():
        break

