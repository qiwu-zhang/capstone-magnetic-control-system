import cv2
import numpy as np
import serial
import time
import os
import subprocess

CAMERA_X=1280
CAMERA_Y=720
serial_port = '/dev/cu.usbmodem1401'
current_x = 0
current_y = 0
arduino_serial = False
BAUD_RATE = 115200
MAX_DELTA = 50
MAX_DELTA_DELAY = 0.6
MAX_1_DELTA = 30
MAX_1_DELTA_DELAY = 0.2
MIN_DELTA = 1.1
max_delta_count = 0
max_delta_1_count = 0
x, y, w, h = 0, 0, 0, 0
first_point_saved = False
second_point_saved = False
track_window = (x, y, w, h)
can_track = False
object_size=0


def get_serial_port_on_mac():
    result = subprocess.run(['ls /dev/cu.usbmodem*', '-l'], capture_output=True, text=True)

    #cmd = "ls /dev/cu.usbmodem*"
    print("return",result)
    global serial_port 
    serial_port = result


def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

def write_ad(x):
    try:
        arduino.write(bytes(x, 'utf-8'))
    except:
        print("arduino write ERROR")
    

def click_event(event, px, py, flags, param):
    global x, y, w, h, first_point_saved,second_point_saved,track_window, can_track, final_video_output
    # Left mouse button release event
    if event == cv2.EVENT_RBUTTONDOWN:
        print("click")
        if first_point_saved:
            w = px-x
            h = py-y
            
            track_window = (x, y, w, h)
            global object_size 
            object_size= w*h
            print(x, y, w, h)
            if (w>0 and h >0):
                first_point_saved = False
                second_point_saved = True
            else:
                first_point_saved = False
                second_point_saved = False

        else:
            x = px
            y = py
            first_point_saved = True
            can_track = False
    # Right mouse button press event
    if event == cv2.EVENT_LBUTTONDOWN:
        print("rright")
        first_point_saved=False
        second_point_saved = False
        can_track = False



# initialize tracker 
def initialize(frame, track_window):
    x, y, w, h = track_window
    # set up the ROI for tracking
    roi = frame[y:y+h, x:x+w]
    #grayscale
    #roi = cv2.cvtColor(roi,cv2.COLOR_GRAY2RGB)
    
    hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsv_roi = roi
    roi_hist = cv2.calcHist([hsv_roi],[0],None ,[180],[0,180])
    roi_hist  = cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)
    return roi_hist, roi


def send_cmd(x):
    write_ad(x+"\r\n")
    return


def track_robot(track_x,track_y,video_center_x,video_center_y):
    global max_delta_count,max_delta_1_count
    delta_x = (video_center_x-track_x)
    delta_y = (video_center_y-track_y)
    #slow down for fast movement
    gain = 1
    if (max_delta_count>0 or abs(delta_x)>MAX_1_DELTA or abs(delta_y)>MAX_1_DELTA):
        max_delta_count+=1
        gain = 10
        if abs(delta_x)>MAX_DELTA or abs(delta_y)>MAX_DELTA:
            delay_counter = 10
        else:
            delay_counter = 5

        if max_delta_count < delay_counter:
            print("max_delay")
            return    
        else:
            max_delta_count=0
    '''
    if (max_delta_1_count>0 or abs(delta_x)>MAX_1_DELTA or abs(delta_y)>MAX_1_DELTA):
        max_delta_1_count+=1
        if max_delta_1_count < 5:
            print("max1delay")
            return    
        else:
            max_delta_1_count=0
    '''
    #skip to reduce noice
    if (abs(delta_x)<MIN_DELTA and abs(delta_y)<MIN_DELTA):
        return
    global current_y,current_x
    current_x=current_x + delta_x*gain
    current_y=current_y + delta_y*gain
    
        #time.sleep(MAX_DELTA_DELAY)
    print(video_center_x,video_center_y,track_x,track_y,"current_x",current_x,"current_y",current_y,"d_x",delta_x,"d_y",delta_y)
    cmd = "mv"+str(" ")+str(current_x)+str(" ")+str(current_y)
    send_cmd(cmd)

    return

def pid_track_robot(track_x,track_y,video_center_x,video_center_y):
    cmd = "pi"+str(" ")+str(track_x)+str(" ")+str(track_y)
    for i in range(10):
        send_cmd(cmd)
    print(cmd)
    return


def nothing(x):
    pass

def on_hello(*args):
	print('hello', 		args)
def on_bye(*args):
	print('bye', 		args)
def on_check(*args):
	print('checkbox', 	args)
def on_radio(*args):
	print('radio',		args)

#################################
def connect_arduino():
    global serial_port
    global arduino,arduino_serial
    try:
        arduino = serial.Serial(port=serial_port, baudrate=BAUD_RATE, timeout=.1)
        arduino_serial = True
        print("arduino connected")
    except:
        print("arduino offline")

def cv2_track():
    global second_point_saved, can_track,track_window
    global current_y,current_x

    windowname = 'Result'
    cv2.namedWindow(windowname)
    cap = cv2.VideoCapture(0)
    cv2.createTrackbar('R','Result',0,255,nothing)
    cv2.createTrackbar('G','Result',0,255,nothing)
    cv2.createTrackbar('B','Result',0,255,nothing)
    


    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    frame_rate = cap.get(cv2.CAP_PROP_FPS)
    color = cap.get(cv2.VIDEOWRITER_PROP_IS_COLOR)
    n_channels = cap.get(cv2.CAP_PROP_CHANNEL)
    print(width, height,frame_rate,color,n_channels)

    cv2.setMouseCallback(windowname, click_event)  # Start the mouse event
    # Setup the termination criteria
    term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 5 )

    mag = 1
    window_size = 200
    crop_frame_x_l = int(CAMERA_X/2-window_size)
    crop_frame_x_h = int(CAMERA_X/2+window_size)
    crop_frame_y_l = int(CAMERA_Y/2-window_size)
    crop_frame_y_h = int(CAMERA_Y/2+window_size)
    new_window_size_x= crop_frame_x_h-crop_frame_x_l
    new_window_size_y= crop_frame_y_h-crop_frame_y_l
    new_window_center_x = new_window_size_x/2
    new_window_center_y = new_window_size_y/2


    print(crop_frame_x_l,crop_frame_x_h,crop_frame_y_l,crop_frame_y_h)
    while True:
        global final_video_output
        ret_it, frame = cap.read()
        frame = frame[crop_frame_y_l:crop_frame_y_h,crop_frame_x_l:crop_frame_x_h]

        # Show the final_video_output
        #final_video_output = frame

        

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = frame
        # Check if 2nd point is also saved then initialize the tracker
        if second_point_saved==True:
            print("init")
            roi_hist, roi = initialize(frame, track_window)
            second_point_saved = False
            can_track = True
        
        # Start tracking
        if can_track == True:
            dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
            # apply camshift to get the new location
            ret, track_window = cv2.CamShift(dst, track_window, term_crit)
            if(ret[1][0]*ret[1][1]>object_size*10):
                can_track = False
            # Draw it on image
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)
            #RotatedRect mass center | width height | clockwise angle
            #print(ret)
            if arduino_serial == True:
                #time.sleep(0.1)
                #track_robot(ret[0][0],ret[0][1],new_window_center_x,new_window_center_y)

                pid_track_robot(ret[0][0],ret[0][1],new_window_center_x,new_window_center_y)


            cv2.imshow('roi', roi)
            final_video_output = cv2.polylines(frame,[pts],True, 255,2)
            
        else:
            final_video_output = frame
            if first_point_saved:
                cv2.circle(final_video_output, (x, y), 5, (0, 0, 255), -1)
                cv2.destroyWindow('roi')
            

        cv2.imshow(windowname,final_video_output)
        if cv2.waitKey(1) == ord('t'):
            if can_track: can_track=False
            else:can_track=True
        if cv2.waitKey(1) == ord('h'):
            print("home")
            current_x = 0
            current_y = 0
            send_cmd("hm")
        if cv2.waitKey(1) == ord('c'):
            print("center")
            current_x = 0
            current_y = 0
            send_cmd("ct")
        if cv2.waitKey(1) == ord('q'):
            break



    cap.release()
    cv2.destroyAllWindows()


get_serial_port_on_mac()
connect_arduino()
cv2_track()
