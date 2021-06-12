# Created by @Saksham Solanki
# This is a program which will control a sophisticated system of lawn mower. 
# It can detect and avoid obstacles through a method never made before. 
# It has a simple GUI which is easy to use. The program has 2 modes, out of 
# which one is Manual Mode --> In this mode the user has to create path by 
# maneuvering the mower around the field through GUI. They can get current 
# stats of sensors through GUI too including lat and longgiven by GPS
# sensor. Then this path will be used on later for automatic maneuvering. 
# Now, the other mode is Automatic mode --> Here the saved path will be 
# used for Automatic maneuvering. Along side this, live feed from the camera 
# will be shown in order to get the exact position and work around of the mower. 
# If the sensors detects any kind off obstacle is detected from right, left 
# or back then it will shut itself down for 10 seconds and will pause all motors 
# and on going functions, also it turn on the alarm for 1 sec and will turn it 
# off to the user let know. After the obstacle is removed it will continue its 
# maneuver. The main feature of the mode is that along side maneuver it can detect 
# obstacles and can surpass them easily. 

#---------------------------------------------- Import Libraries ----------------------------------------------# 
import numpy as np # Library used for some mathematical calculations
import tkinter as tk # library for GUI
from PIL import Image, ImageTk #Library to process video processing for live feed
import multiprocessing #library for running multiple process at same time
from cv2 import cv2# library for video processing
import RPi.GPIO as GPIO # library to control and get inputs from GPIO pins of pi
import time # library to get exact time for moving
import serial # lirbrary to get data from GPS
import adafruit_gps # library to parse data from GPS
import os # library for file handling to create paths
import psutil # library which will be used to pause process (a part of moving algorithm)
from dual_g2_hpmd_rpi import Motor, motors, MAX_SPEED # library to control motors
import math
#--------------------------------------------------------------------------------------------------------------#

#-------------------------------------------- Init Speed for Motors -------------------------------------------#
# These Values will be used as the forward and as the backward values
test_forward_speeds = list(range(0, MAX_SPEED, 1)) + \
[MAX_SPEED] * 200 + list(range(MAX_SPEED, 0, -1)) + [0]# Forward value

test_reverse_speeds = list(range(0, -MAX_SPEED, -1)) + \
[-MAX_SPEED] * 200 + list(range(-MAX_SPEED, 0, 1)) + [0]# Backward value
#--------------------------------------------------------------------------------------------------------------#
main_time = 0
command = ""
#____________________________________________ Output of all sensors ___________________________________________#
class Sensors:
#-------------------------------------------- Ultrasonic Sensor_1 ---------------------------------------------#
    def Ultrasonic_sensor_1(): # This is the function to get distance from Ultrasonic sensor placed forward
        GPIO.setmode(GPIO.BOARD)
        TRIG = 22
        ECHO = 38
        i=0
        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)
        GPIO.output(TRIG, False)
        pulse_end = 0
        pulse_start = 0
        time.sleep(2)
        try:
            while True:
                GPIO.output(TRIG, True)
                time.sleep(0.00001)
                GPIO.output(TRIG, False)
                while GPIO.input(ECHO)==0:
                    pulse_start = time.time()
                while GPIO.input(ECHO)==1:
                    pulse_end = time.time()
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17150
                distance = round(distance+1.15, 2)
                return distance
        except:
            pass
            GPIO.cleanup()
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------------- Ultrasonice Sensor_2 -------------------------------------------#
    def Ultrasonic_sensor_2(): # This is the function to get distance from Ultrasonic sensor placed forward
        GPIO.setmode(GPIO.BOARD)
        TRIG = 36
        ECHO = 40
        i=0
        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)
        GPIO.output(TRIG, False)
        pulse_start = 0
        pulse_end = 0
        time.sleep(2)
        try:
            while True:
                GPIO.output(TRIG, True)
                time.sleep(0.00001)
                GPIO.output(TRIG, False)
                while GPIO.input(ECHO)==0:
                    pulse_start = time.time()
                while GPIO.input(ECHO)==1:
                    pulse_end = time.time()
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17150
                distance = round(distance+1.15, 2)
                return distance
        except:
            pass
            GPIO.cleanup()
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- IR Sensor 1 ------------------------------------------------#
    def IR_1(): #This is the function to get value of object being placed or not, from IR sensor 1 placed forward
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(11,GPIO.IN)
        while(True):
            if GPIO.input(11):
                return GPIO.input(11)
            else:
                return False
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- IR Sensor 2 ------------------------------------------------#
    def IR_2(): #This is the function to get value of object being placed or not, from IR sensor 2 placed Backward
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(37,GPIO.IN)
        while(True):
            if GPIO.input(37):
                return GPIO.input(37)
            else:
                return False
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- IR Sensor 3 ------------------------------------------------#
    def IR_3(): #This is the function to get value of object being placed or not, from IR sensor 3 placed Left
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(35,GPIO.IN)
        while(True):
            if GPIO.input(35):
                return GPIO.input(35)
            else:
                return False
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- IR Sensor 4 ------------------------------------------------#
    def IR_4(): #This is the function to get value of object being placed or not, from IR sensor 4 placed Right
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(13,GPIO.IN)
        while(True):
            if GPIO.input(13):
                return GPIO.input(13)
            else:
                return False
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------- Set values of ultrasonic sensors ----------------------------------------#
    def get_us_sensors(): # This function will alter the value of Ultrasonic sensors in GUI in manual mode
        s1 = str(Sensors.Ultrasonic_sensor_1())
        s2 = str(Sensors.Ultrasonic_sensor_2())
        Ultrasonic_s_1.config(text="US Sensor_1 = "+s1)
        Ultrasonic_s_2.config(text="US Sensor_2 = "+s2)
#---------------------------------------------------------------------------------------------------------------#
#----------------------------------------- Set values of IR Sensors --------------------------------------------#
    def get_ir_sensors(): # This function will alter the value of IR sensors in GUI in manual mode 
        i1 = Sensors.IR_1()
        if i1 == True:
            IR_1.config(text="IR Sensor_1 = Object Detected")
        else: IR_1.config(text="IR Sensor_1 = Clear")
        i2 = Sensors.IR_2()
        if i2 == True:
            IR_2.config(text="IR Sensor_2 = Object Detected")
        else: IR_2.config(text="IR Sensor_2 = Clear")
        i3 = Sensors.IR_3()
        if i3 == True:
            IR_3.config(text="IR Sensor_3 = Object Detected")
        else: IR_3.config(text="IR Sensor_3 = Clear")
        i4 = Sensors.IR_4()
        if i4 == True:
            IR_4.config(text="IR Sensor_4 = Object Detected")
        else: IR_4.config(text="IR Sensor_4 = Clear")
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------- Set Lat and Long values of GPS ---------------------------------------#
    def get_GPS(): # This function will alter the value of Lat and Long in GUI in manual mode
        lat, long = Motor_Controlling.GPS()
        lat, long = str(lat), str(long)
        Gps.config(text="Lat, Long = " + lat + ", " + long)
#---------------------------------------------------------------------------------------------------------------#
#_______________________________________________________________________________________________________________#

#___________________________________________________ Live Feed _________________________________________________#
class MainWindow():
    # Here I have made a simple class which will display live feed from the camera in automatic mode. How this
    # works is that it takes image, convert it to the right format and then display it. This will happen every 
    # 20 millisecond which will make it look like as a video and won't put much processing power into use.
#--------------------------------------------- Initialize class ------------------------------------------------#
    def __init__(self, window, cap):
        self.window = window
        self.cap = cap
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.interval = 20 # Interval in ms to get the latest frame
        # Create canvas for image
        self.canvas = tk.Canvas(self.window, width=self.width, height=self.height)
        self.canvas.grid(row=0, column=0)
        # Update image on canvas
        self.update_image()
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------------------ Update Video -------------------------------------------------#
    def update_image(self):
        # Get the latest frame and convert image format
        self.image = cv2.cvtColor(self.cap.read()[1], cv2.COLOR_BGR2RGB) # to RGB
        self.image = Image.fromarray(self.image) # to PIL format
        self.image = ImageTk.PhotoImage(self.image) # to ImageTk format
        # Update image
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image)
        # Repeat every 'interval' ms
        try:
            self.window.after(self.interval, self.update_image)
        except: 
            pass
#_______________________________________________________________________________________________________________#

#-------------------------------------------------- Exception class --------------------------------------------#
# This is a class which will be used for creating exception if the motors get any issues
class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num
#---------------------------------------------------------------------------------------------------------------#

#_________________________________ Functions which will control all motors _____________________________________#
class Motor_Controlling:
#---------------------------------------------- Get Lat and Long -----------------------------------------------#
    def GPS():
        uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        gps = adafruit_gps.GPS(uart, debug=False)
        gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        gps.send_command(b"PMTK220,1000")
        last_print = time.monotonic()
        while True:
            gps.update()
            current = time.monotonic()
            if current - last_print >= 1.0:
                last_print = current
                if not gps.has_fix:
                    continue
            lat = gps.latitude
            long = gps.longitude
            try:
                lat = round(lat, 5)
                long = round(long, 5)
            except: 
                continue
            if lat == None or long == None:
                continue
            else:
                return lat, long
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------ Timer function to run multiple things ------------------------------------#
    def timer(sleep_duration=1):
        while sleep_duration != 0:
            time.sleep(1)
            sleep_duration -= 1
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------- Raise fault if there is an exception-------------------------------------#
    def raiseIfFault():
        if motors.motor1.getFault():
            raise DriverFault(1)
        if motors.motor2.getFault():
            raise DriverFault(2)
#-------------------------------------------------- Right direction --------------------------------------------#
# First of all it will take a right and move forward till 1 second. After that it will proceed to move 
# forward untill path is changed
    def Right():
        def right():
            for s in test_reverse_speeds:
                motors.motor1.setSpeed(s)
                Motor_Controlling.raiseIfFault()
                time.sleep(0.002)
        timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(3,))
        timer_thread.start()
        check = True
        while check:
            if timer_thread.is_alive() == False:
                Motor_Controlling.stop()
                check = False
            elif timer_thread.is_alive() == True:
                right()
            else:
                continue
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- Left direction ---------------------------------------------#
# First of all it will take a left and move forward till 2 second. After that it will proceed to move
# forward untill path is changed
    def Left():
        def left():
            for s in test_forward_speeds:
                motors.motor2.setSpeed(s)
                Motor_Controlling.raiseIfFault()
                time.sleep(0.002)
        timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(3,))
        timer_thread.start()
        check = True
        while check:
            if timer_thread.is_alive() == False:
                Motor_Controlling.stop()
                check = False
            elif timer_thread.is_alive() == True:
                left()
            else: continue
#---------------------------------------------------------------------------------------------------------------#
#--------------------------------------------- Reverse right and left ------------------------------------------#
# These functions are not used in the program but still have been made in case a require for them persists in 
# future
    def reverse_right():
        for s in test_reverse_speeds:
            motors.motor1.setSpeed(s)
            Motor_Controlling.raiseIfFault()
            time.sleep(0.2) 
  
    def reverse_left():
        for s in test_forward_speeds:
            motors.motor2.setSpeed(s)
            Motor_Controlling.raiseIfFault()
            time.sleep(0.002)    
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------------- Backward direction ---------------------------------------------#
    def backward():
        motors.setSpeeds(0, 0)
        for (s,s2) in zip(test_forward_speeds, test_reverse_speeds):
            motors.motor1.setSpeed(s)
            motors.motor2.setSpeed(s2)
            Motor_Controlling.raiseIfFault()
            time.sleep(0.002)
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------------- Forward direction ----------------------------------------------#
    def forward():
        motors.setSpeeds(0, 0)
        for (s, s2)  in zip(test_reverse_speeds, test_forward_speeds):
            motors.motor1.setSpeed(s)
            motors.motor2.setSpeed(s2)
            Motor_Controlling.raiseIfFault()
            time.sleep(0.002)
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------------------- Stop motors -------------------------------------------------#
    def stop():
        global main_time
        global command
        data = Manual.path_data()
        if main_time != 0:
            time_2 = int(time.time()-main_time)
            with open(n_path, 'w') as f:
                f.write(data+'\n'+command+'\n'+str(int(time_2)))
        motors.forceStop()
        motors.disable()
        motors.enable()
    def stop_2():
        motors.forceStop()
        motors.disable()
        motors.enable()
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------------ Automatic mode initializer -----------------------------------------#
# 3 processes will run together where one is to move according to the given path, one is to check for all values
# of all sensors and the last one is for obstacle avoiding. All three of them will run parallely and will make 
# sure to pause each one if any kind of disturbance is found
    def main():
        p1 = multiprocessing.Process(target=Automatic.file_parser, args=(n_path, ))
        p2 = multiprocessing.Process(target=Motor_Controlling.check_forward)
        p3 = multiprocessing.Process(target=Motor_Controlling.check_sensors)
        global p_0
        global p_1
        global p_2
        p_0 = psutil.Process(p1.pid)
        p_1 = psutil.Process(p2.pid)
        p_2 = psutil.Process(p3.pid)
        p1.start()
        p2.start()
        p3.start()
        p1.join()
        p2.join()
        p3.join()
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------------- Sensors output checking -------------------------------------------#
# If any of the sensors gets blocked, it will let out a siren for 1 sec and then turn off itself for 10 seconds
# and will continue doing this until the obstacle is removed from the sides or back.
    def check_sensors():
        while Sensors.IR_3() == True or Sensors.IR_4() == True or (Sensors.Ultrasonic_sensor_2() <=10 and Sensors.IR_2() == True):
            p_0.suspend()
            p_1.suspend()
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(19, GPIO.OUT)
            GPIO.output(19, False)
            GPIO.setup(21, GPIO.OUT)
            GPIO.output(21, True)
            time.sleep(1)
            GPIO.setup(21, GPIO.OUT)
            GPIO.output(21, False) 
            time.sleep(10)
        GPIO.setup(19, GPIO.OUT)
        GPIO.output(19, True)
        p_0.resume()
        p_1.resume()
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------------- Obstacle avoiding ----------------------------------------------#
# If it finds anything in front of it, it will automatically stop itself for 10 seconds and if 
# the obstacle is still there then it will first of all check if there is an obstacle on the 
# right or not, if there is then it will check for left and if there is too then it will set 
# the alarm on until it is removed. Else if there is not any one on the right, it will take a 
# turn on right, move forward till the corner of the obstacle, and will also store the time it 
# took to reach the corner. After this it will take a left and move forward until it reaches
# the end of the obstacle, after that it will take left again and will move forward till the 
# time it took it at the start to reach corner of the obstacle. After this it will take a right 
# and will then resume the process of maneuvering. Also if there is something on left it will 
# repeat the same process from the left side.
    def check_forward():
        while True:
            s1 = Sensors.Ultrasonic_sensor_1()
            s2 = Sensors.IR_1()
            if s1 <= 10 and s2 == True:
                p_0.suspend()
                p_2.suspend()
                GPIO.setmode(GPIO.BOARD)
                GPIO.setwarnings(False)
                GPIO.setup(19, GPIO.OUT)
                GPIO.output(19, False)
                time.sleep(10)
                if Sensors.IR_4() == False:
                    Motor_Controlling.Right()
                    start = time.time()
                    while Sensors.IR_3() == True:
                        Motor_Controlling.forward()
                    end = int((time.time() - start))
                    while Sensors.IR_3() == False:
                        Motor_Controlling.Left()
                        Motor_Controlling.forward()
                    while Sensors.IR_3() == True:
                        Motor_Controlling.forward()
                    Motor_Controlling.Left()
                    timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(end,))
                    timer_thread.start()
                    check = True
                    while check:
                        if timer_thread.is_alive() == True:
                            Motor_Controlling.forward()
                        elif timer_thread.is_alive() == False:
                            check = False
                        else: continue
                    Motor_Controlling.Right()
                    Motor_Controlling.stop()
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(19, GPIO.OUT)
                    GPIO.output(19, True)
                    p_0.resume()
                    p_2.resume()
                elif Sensors.IR_3() == False:
                    Motor_Controlling.Left()
                    start = time.time()
                    while Sensors.IR_4() == True:
                        Motor_Controlling.forward()
                    end = int((time.time() - start))
                    while Sensors.IR_4() == False:
                        Motor_Controlling.Right()
                        Motor_Controlling.forward()
                    while Sensors.IR_4() == True:
                        Motor_Controlling.forward()
                    Motor_Controlling.Right()
                    timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(end,))
                    timer_thread.start()
                    check = True
                    while check:
                        if timer_thread.is_alive() == True:
                            Motor_Controlling.forward()
                        elif timer_thread.is_alive() == False:
                            check = False
                        else: continue
                    Motor_Controlling.Left()
                    Motor_Controlling.stop()   
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(19, GPIO.OUT)
                    GPIO.output(19, True)
                    p_0.resume()
                    p_2.resume()
                else:
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(21, GPIO.OUT)
                    GPIO.output(21, True)
            else: 
                continue
#---------------------------------------------------------------------------------------------------------------#
#_______________________________________________________________________________________________________________#

#______________________________________________ Automaic maneuvering ___________________________________________#
class Automatic:
#------------------------------------------------- File reader -------------------------------------------------#
# The function will read the file, convert lines to commands and will decide accordingly to it
    def file_parser(file):
        with open(file, 'r') as f:
            path = f.read()
            f.close()
        paths = list(path.split(","))
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(19, GPIO.OUT)
        GPIO.output(19, True)
        for way_point in paths:
            way_points = list(way_point.split("\n"))
            try: 
                s = way_points[2]
                del(way_points[0])
            except:
                pass
            try:
                auto_time = int(way_points[1])
            except: 
                print("maneuver completed")
                exit()
            if "Forward" in way_points[0]:
                timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(auto_time,))
                timer_thread.start()
                check = True
                while check:
                    if timer_thread.is_alive() == True:
                        Motor_Controlling.forward()
                    elif timer_thread.is_alive() == False:
                        check = False
                    else: continue
            elif "Backward" in way_points[0]:
                timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(auto_time,))
                timer_thread.start()
                check = True
                while check:
                    if timer_thread.is_alive() == True:
                        Motor_Controlling.backward()
                    elif timer_thread.is_alive() == False:
                        check = False
                    else: continue
            elif "Right" in way_points[0]:
                timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(auto_time,))
                timer_thread.start()
                check = True
                while check:
                    if timer_thread.is_alive() == True:
                        Motor_Controlling.Right()
                    elif timer_thread.is_alive() == False:
                        check = False
                    else: continue
            elif "Left" in way_points[0]:
                timer_thread = multiprocessing.Process(target=Motor_Controlling.timer, args=(auto_time,))
                timer_thread.start()
                check = True
                while check:
                    if timer_thread.is_alive() == True:
                        Motor_Controlling.Left()
                    elif timer_thread.is_alive() == False:
                        check = False
                    else: continue
            else:
                pass
#---------------------------------------------------------------------------------------------------------------#
#_______________________________________________________________________________________________________________#

#______________________________________________ Manual maneuvering _____________________________________________#
class Manual:
#------------------------------------------ Create path and directory ------------------------------------------#
# The function will first here try to read a file, if it fails it will create the path, else if it is able to do
# it then it will simply just return the absolute path.
    def path_file():
        try:
            with open("paths/path0.txt", 'r') as f:
                f.close()
            fin_path = "paths/path0.txt"
            return fin_path
        except:
            if not os.path.exists("paths"):
                os.mkdir("paths")
            with open("paths/path0.txt", 'w+') as f:
                f.close()
            fin_path = "paths/path0.txt"
            return fin_path                      
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- Init path --------------------------------------------------#
    def init():
        global n_path
        n_path = Manual.path_file()
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------------- Read path and return it -------------------------------------------#
    def path_data():
        with open(n_path) as f:
            data = f.read()
            f.close()
        return data
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------------- Forward direction ----------------------------------------------#
    def forward():
        global main_time
        global command
        data = Manual.path_data()
        if main_time != 0:
            time_2 = (time.time()-main_time)
            time_2 = math.ceil(time_2)
            print(time_2)
            with open(n_path, 'w') as f:
                f.write(data+'\n'+command+'\n'+str(int(time_2))+',')
                f.close()

        command = "Forward"
        main_time = time.time()
        Motor_Controlling.forward()
#---------------------------------------------------------------------------------------------------------------#
#---------------------------------------------- Backward direction ---------------------------------------------#
    def backward():
        global main_time
        global command
        data = Manual.path_data()
        if main_time != 0:
            time_2 = (time.time()-main_time)
            time_2 = math.ceil(time_2)
            print(time_2)
            with open(n_path, 'w') as f:
                f.write(data+'\n'+command+'\n'+str(int(time_2))+',')
                f.close()

        command = "Backward"
        main_time = time.time()
        Motor_Controlling.backward()
#---------------------------------------------------------------------------------------------------------------#
#----------------------------------------------- Right direction -----------------------------------------------#
    def right():
        global main_time
        global command
        data = Manual.path_data()
        if main_time != 0:
            time_2 = (time.time()-main_time)
            time_2 = math.ceil(time_2)
            print(time_2)
            with open(n_path, 'w') as f:
                f.write(data+'\n'+command+'\n'+str(int(time_2))+',')
                f.close()

        command = "Right"
        main_time = time.time()
        Motor_Controlling.Right()
#---------------------------------------------------------------------------------------------------------------#
#----------------------------------------------- Left direction ------------------------------------------------#
    def left():
        global main_time
        global command
        data = Manual.path_data()
        if main_time != 0:
            time_2 = (time.time()-main_time)
            time_2 = math.ceil(time_2)
            print(time_2)
            with open(n_path, 'w') as f:
                f.write(data+'\n'+command+'\n'+str(int(time_2))+',')
                f.close()

        command = "Left"
        main_time = time.time()
        Motor_Controlling.Left()
#---------------------------------------------------------------------------------------------------------------#
#-------------------------------------------- Reverse left and right -------------------------------------------#
# # These functions are not used in the program but still have been made in case a require for them persists in 
# # future
#     def reverse_left():
#         lat,long = Motor_Controlling.GPS()
#         Motor_Controlling.reverse_left()
#         data = Manual.path_data()
#         with open(n_path, 'w+') as f:
#             f.write(data + '\n' + "Reverse_L" + '\n' + str(lat) + "/" + str(long) + ",")
#             f.close()
#     def reverse_right():
#         lat,long = Motor_Controlling.GPS()
#         Motor_Controlling.reverse_right()
#         data = Manual.path_data()
#         with open(n_path, 'w+') as f:
#             f.write(data + '\n' + "Reverse_R" + '\n' + str(lat) + "/" + str(long) + ",")
#             f.close()
# #---------------------------------------------------------------------------------------------------------------#
#_______________________________________________________________________________________________________________#

#______________________________________________________ GUI ____________________________________________________#
#---------------------------------------- Live feed and Automatic frame ----------------------------------------#
def Live_feed():
    root = tk.Tk()
    root.title("Automatic Mode")
    try: MainWindow(root, cv2.VideoCapture(0))
    except: pass
    GPIO.cleanup()
    maneuver = tk.Button(root, text="Start Maneuver",command=lambda:[Manual.init(), Motor_Controlling.main()]).place(relx=0.4, rely=0.9)
    Stop = tk.Button(root, text="Stop", command=lambda:Motor_Controlling.stop_2()).place(relx=0.3, rely=0.9)
    Manual_mode = tk.Button(root, text="Manual", command=lambda:[root.destroy(),Manual.init(), Manual_frame()]).place(relx=0.61, rely=0.9)
    root.mainloop()
#---------------------------------------------------------------------------------------------------------------#
#------------------------------------------------- Manual Frame ------------------------------------------------#
def Manual_frame():
    #GPIO.cleanup()
    m_frame = tk.Tk()
    m_frame.title("Manual Mode")
    m_frame.geometry("600x650")
    Stop = tk.Button(m_frame, text="Stop", command=lambda:Motor_Controlling.stop()).place(relx=0.35)
    automatic = tk.Button(m_frame, text="Automatic", command=lambda:[m_frame.destroy(), Live_feed()]).place(relx=0.46)
    Up = tk.Button(m_frame, text="Forward", background="red", command=lambda:Manual.forward()).place(relx=0.43, rely=0.2)
    Down = tk.Button(m_frame, text="Reverse", background="red", command=lambda:Manual.backward()).place(relx=0.43, rely=0.36)
    Left = tk.Button(m_frame, text=" Left", background="blue", command=lambda:Manual.left()).place(relx=0.3, rely=0.28)
    Right = tk.Button(m_frame, text="Right", background="blue", command=lambda:Manual.right()).place(relx=0.6, rely=0.28)
    Get_stats = tk.Button(m_frame, text="Get Status", command=lambda:[Sensors.get_us_sensors(), Sensors.get_ir_sensors(), Sensors.get_GPS()]).place(relx=0.1, rely=0.6)
    global IR_1
    global IR_2
    global IR_3
    global IR_4
    global Ultrasonic_s_1
    global Ultrasonic_s_2
    global Gps
    IR_1 = tk.Label(m_frame, text="IR Sensor_1 = 0")
    IR_1.place(relx=0.1, rely=0.69)
    IR_2 = tk.Label(m_frame, text="IR Sensor_2 = 0")
    IR_2.place(relx=0.1, rely=0.72)
    IR_3 = tk.Label(m_frame, text="IR Sensor_3 = 0")
    IR_3.place(relx=0.1, rely=0.75)
    IR_4 = tk.Label(m_frame, text="IR Sensor_4 = 0")
    IR_4.place(relx=0.1, rely=0.78)
    Ultrasonic_s_1 = tk.Label(m_frame, text="US Sensor_1 = 0")
    Ultrasonic_s_1.place(relx=0.1, rely=0.82)
    Ultrasonic_s_2 = tk.Label(m_frame, text="US Sensor_2 = 0")
    Ultrasonic_s_2.place(relx=0.1, rely=0.85)
    Gps = tk.Label(m_frame, text="Lat, Long = 0, 0")
    Gps.place(relx=0.1, rely=0.88)
    m_frame.mainloop()
#----------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------- Main Frame --------------------------------------------------#
def Dashboard():
    window_2 = tk.Tk()
    window_2.title("Lawn Mower")
    window_2.geometry("500x400")
    manual = tk.Button(window_2, text=" Manual ", command=lambda:[window_2.destroy(), Manual.init(), Manual_frame()]).place(relx=0.52, rely=0.4)
    automatic = tk.Button(window_2, text="Automatic", command=lambda:[window_2.destroy(), Live_feed()]).place(relx=0.33, rely=0.4)
    window_2.mainloop()
#----------------------------------------------------------------------------------------------------------------#
#________________________________________________________________________________________________________________#

#----------------------------------------------- Init Main frame ------------------------------------------------#
if __name__ == '__main__':
    Dashboard()
#----------------------------------------------------------------------------------------------------------------#