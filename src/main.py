#https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import collections
import serial
import pyautogui
import keyboard
import glob
from math import  pi, atan, atan
from scipy.optimize import lsq_linear
import time

direction_global = "none"
direction_global_nxt = "left"
constant_speed_turn = 2
constant_speed_straight = 8
camera_index = 0
center_tolerance = 100
display_camera = 0
robot_mode = int(input(">> robot mode: "))#1 #1:pingpong 2:follower
global_time = time.time()
threshold_time = 18
action_mode = "wander"
# BAUD_RATES = 9600
# for port_i in range(20):
#     try:
#         COM_PORT = '/dev/ttyACM{}'.format(port_i)
#         ser = serial.Serial(COM_PORT, BAUD_RATES)
#     except:
#         pass

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

blueLower = (101,50,38)
blueUpper = (110,255,255)

# orangeLower = (10, 80, 200)
# orangeUpper = (25, 255, 255)

orangeLower = (10, 120, 210)
orangeUpper = (20, 255, 255)

#255 197 106 bright RGB,  18 149 255 HSV
#219 117 26  dark RGB, 14 225 219 HSV
# orangeUpper = np.uint8([[[106, 197, 255]]])
# orangeLower = np.uint8([[[26, 117, 219]]])
# hsvOrange1 = cv2.cvtColor(orangeUpper, cv2.COLOR_BGR2HSV)
# hsvOrange1 = cv2.cvtColor(orangeLower, cv2.COLOR_BGR2HSV)

target_lower = orangeLower
target_upper = orangeUpper

pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam

vs = VideoStream(src=camera_index).start()


# allow the camera or video file to warm up
time.sleep(2.0)
global_time = time.time()

def action_taker():
    global direction_global
    global direction_global_nxt
    global global_time
    global action_mode
    if(direction_global_nxt == "right" ):
        if(direction_global == 'right'):
            # print("keep going right")
            pass
        else:
            pyautogui.press("s")
            for i in range(constant_speed_turn):
                pyautogui.press("d")
            # print("start going right")
        direction_global = direction_global_nxt
        global_time = time.time()
        
    elif(direction_global_nxt == "left" ):
        if(direction_global == 'left'):
            pass
        else:
            pyautogui.press("s")
            for i in range(constant_speed_turn):
                pyautogui.press("a")
        direction_global = direction_global_nxt
        action_mode = "action"
        # global_time = time.time()
        # print("going left")
    elif(direction_global_nxt == "straight" ):
        if(direction_global == 'straight'):
            pass
        else:
            pyautogui.press("s")
            for i in range(constant_speed_straight):
                pyautogui.press("x")
        direction_global = direction_global_nxt
        action_mode = "action"
        global_time = time.time()
    elif(direction_global_nxt == "stop"):
        pyautogui.press("s")
        direction_global = direction_global_nxt
        action_mode = "action"
        global_time = time.time()

def pick_up_ball():
    global direction_global
    global direction_global_nxt
    pyautogui.press("x")
    pyautogui.press("x")
    time.sleep(3)
    pyautogui.press("s")
    time.sleep(2)
    direction_global_nxt = "right"

def Action_Decider(midpoint_x, midpoint_y, center, center_tolerance):
    global direction_global_nxt
    if((midpoint_x-center_tolerance)<center[1] and center[1]<(midpoint_x+center_tolerance)):
        direction_global_nxt = "straight"
    elif(center[1]<(midpoint_x-center_tolerance)):
        direction_global_nxt = "left"
    elif((midpoint_x+center_tolerance)<center[1]):
        direction_global_nxt = "right"
    else:
        direction_global_nxt = "left"
    return 

def getLinearEquation(p1x,p1y,p2x,p2y): 
    sign =1
    a=p2y-p1y
    if (a < 0):
        sign =-1
        a=sign*a
    b=sign*(p1x-p2x)
    c=sign*(p1y*p2x-p1x*p2y)
    return [a,b,c]

def Calculate_Distance(coeff, mark_radius):
    # ax+by+c=0
    [a,b,c] = coeff
    return round((-c-b*mark_radius)/a, 1)

def servo_action(ser):
    ser.write(b'ACTION\n')
    time.sleep(6)
    return

def count_down(time_count):
    for i in range(time_count):
        print(">> take control in {} secs".format(time_count-i))
        time.sleep(1)
    return

def switch_mode():
    global robot_mode
    while True:
        if(keyboard.is_pressed("2")):
            robot_mode = 2
            print("@@@@ swith to mode 2")
            return "keep"
        elif(keyboard.is_pressed("1")):
            robot_mode = 1
            print("@@@@ swith to mode 1")
            return "keep"
        else:
            return "stop"

def find_ports():
    ports = glob.glob('/dev/ttyACM[1-9]*')
    res = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            res.append(port)
        except:
            print('wrong port!')
    return res

def ToA_xyz(distances_to_anchors, anchor_positions):
    if len(distances_to_anchors) <= 3:
         return('only connect to ' + str(len(distances_to_anchors)) + ' uwb anchors, ')
    distances_to_anchors,anchor_positions = np.array(distances_to_anchors),np.array(anchor_positions)
    anchor_offset = anchor_positions[0]
    anchor_positions = anchor_positions[1:] - anchor_offset
    K = np.sum(np.square(anchor_positions), axis=1)  
    squared_distances_to_anchors = np.square(distances_to_anchors)
    squared_distances_to_anchors = (squared_distances_to_anchors - squared_distances_to_anchors[0])[1:]
    b = (K - squared_distances_to_anchors) / 2.
    res = lsq_linear(anchor_positions, b, lsmr_tol='auto', verbose=0)
    position = res.x + anchor_offset
    # print(position)
    return [position[0],position[1]]

def anchor_selection(distances_to_anchors, anchor_positions):
    distances_to_anchors, anchor_positions = np.array(distances_to_anchors), np.array(anchor_positions)
    disconnect = []
    for i in range(len(anchor_positions)):
        if distances_to_anchors[i] <= 0:
           disconnect.append(i)
    distances_to_anchors = np.delete(distances_to_anchors,disconnect,axis=0)
    anchor_positions = np.delete(anchor_positions,disconnect,axis=0)

    return distances_to_anchors,anchor_positions

def UWB_pos(position):
    try:
        X, Y = position[0],position[1]
        ang = atan(Y/X)*180/pi
        
        if(Y>0 and X>0):ang = ang
        elif(Y>0 and X<0):ang = ang + 180
        elif(Y<0 and X>0):ang = ang
        elif(Y<0 and X<0):ang = ang + 180
        else:ang = ang

        ang = ang%360

        if ang >= 180 and ang <= 360:
            ang -= 360

        dis = (X**2+Y**2)**(0.5)

        return ang, dis
    
    except TypeError:
        print('square_pos function:TypeError angle is none!!!')
    
    except ValueError:
        print('square_pos function: Math domain error! Probably only got 2 distance!!!')

frame = vs.read()
frame = frame[1] if args.get("video", False) else frame
frame = imutils.resize(frame, width=600)
height = frame.shape[0]
width = frame.shape[1]
midpoint_x = width/2
midpoint_y = height/2


# (15, 51.4) (30, 30.1) (45, 18.9)
real_ball_radius = 3.4 #cm
ball_radius_at_mark1 = 54.1 #no unit
length_mark1 = 15 #cm
ball_radius_at_mark2 = 18.9 #no unit
length_mark2 = 45 #cm
dist_coeff = getLinearEquation(length_mark1, ball_radius_at_mark1, length_mark2, ball_radius_at_mark2)

angle_range = [10,-10]
follow_dis = 0.8

count_down(3)
global_time  = time.time()
if(robot_mode==1): #pingpong
    while True:
        try:
            if(direction_global == 'left'):
                if(time.time()-global_time > threshold_time):
                    robot_mode = 2
                    break
                else:
                    print("time count")
                    print(time.time()-global_time)
            # handle the frame from VideoCapture or VideoStream
            frame = vs.read()
            frame = frame[1] if args.get("video", False) else frame

            if frame is None:
                break

            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, target_lower, target_upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            action_taker()
            
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                # c = max(cnts, key=cv2.contourArea)
                for c in cnts:
                    # c = cv2.convexHulfl(c)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # only proceed if the radius meets a minimum size
                    if radius > 1:
                        print(radius)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        cv2.drawContours(frame, c, -1, (0,255,0), 3)
                    
                        distance = Calculate_Distance(dist_coeff, radius)
                        Action_Decider(midpoint_x, midpoint_y, (y, x), center_tolerance)
                        action_taker()

                        if(radius > 25):# and direction_global=="straight"):
                            print("very close")
                            pick_up_ball()

            elif(len(cnts)==0):
                direction_global_nxt = "left"

            pts.appendleft(center)

            if(display_camera):
                cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break
            # time.sleep(0.5)
        except KeyboardInterrupt:
            print("pending")
            key = switch_mode()
            if(key=="stop"):
                vs.release()
                cv2.destroyAllWindows()
            else:
                print("stop")
                pass
if(robot_mode==2):
    #get action from uwb code
    direction_global_nxt = "stop"
    action_taker()
    UWB_port = find_ports()
    ser_UWB = serial.Serial(UWB_port[0], baudrate = 115200, timeout=0.05)
    dis_queue = collections.deque(maxlen=1)
    count_to_left = 0
    count_to_right = 0
    count_to_stop = 0
    count_to_straight = 0
    threshold = 6
    constant_speed_turn = 5
    constant_speed_straight = 10
    while True:
        forward = "stay"
        turn = "Don't turn"
        try:
            
            # time.sleep(0.1)
            if ser_UWB.in_waiting > 0:
                rx = ser_UWB.readline().decode('utf-8')
                if(rx != ' ' and rx.find('mc') >= 0):
                    dis = rx.split(' ')
                    dis_array = np.array([(int(dis[2],16)),(int(dis[3],16)), (int(dis[4],16)), (int(dis[5],16))])/1000.0
                    dis_array = dis_array -0.65
                    # print(dis_array)
                    dis_queue.clear()
                    dis_queue.append(dis_array)
                    # print("queue",dis_queue)
                    anchor_positions = [[0.215,0.215,0],[0.215,-0.215,0],[-0.215,-0.215,0],[-0.215,0.215,0]]
                    dis_array, anchor_positions = anchor_selection(dis_queue[0], anchor_positions)
                    position = ToA_xyz(dis_array, anchor_positions)
                    # print("position",position)
                    if type(position) == list:
                        ang,dis = UWB_pos(position)
                        # print("anchor distance : {}".format(dis_array))
                        print("angle : {}, distance : {}".format(ang,dis))
                        if ang < angle_range[0] and ang > angle_range[1]:
                            turn = "Don't turn"
                            count_to_right = 0 
                            count_to_left = 0
                            if dis >= follow_dis:
                                count_to_straight += 1
                                count_to_stop = 0
                                if count_to_straight >= threshold:
                                    count_to_straight = 0
                                    direction_global_nxt = "straight"
                            if dis < follow_dis:
                                count_to_stop += 1
                                count_to_straight = 0
                                if count_to_stop >= threshold:
                                    count_to_stop = 0
                                    direction_global_nxt = "stop"
                                    forward = "Move Forward"
                            action_taker()
                                # time.sleep(0.5)
                        elif ang > angle_range[0]:
                            count_to_left += 1
                            count_to_stop = 0 
                            count_to_right = 0
                            count_to_straight = 0
                            if count_to_left >= threshold:
                                direction_global_nxt = "left"
                                turn = "Turn Left"
                                forward = "Stay"
                                count_to_left = 0
                                action_taker()
                        elif ang < angle_range[1]:
                            count_to_right += 1
                            count_to_stop = 0 
                            count_to_left = 0
                            count_to_straight = 0
                            if count_to_right >= threshold:
                                direction_global_nxt = "right"                           
                                turn = "Turn Right"
                                forward = "Stay"
                                count_to_right = 0
                                action_taker()
                        # print(forward,turn)
        except ValueError:
            print('ValueError')
        except IndexError:
            print('IndexError')
        # action_taker()
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()
else:
    vs.release()
cv2.destroyAllWindows()

