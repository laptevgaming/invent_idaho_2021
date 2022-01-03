#importing the libraries that will be used
import cv2
import os
import RPi.GPIO as GPIO
from time import sleep
from threading import Thread
from pynput import keyboard
from pynput.keyboard import Listener, Key, KeyCode
import numpy

#initializing the servo motor and variables
thread1 = False
thread2 = False
trap = True
key_input = list()
GPIO.setwarnings(False)
servoPIN = 18
trigger_blocked = False
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
pwm = GPIO.PWM(servoPIN, 50)
code = [KeyCode.from_char('1'), KeyCode.from_char('2'), KeyCode.from_char('3'), KeyCode.from_char('4')]
pwm.start(0)

#activates when a key is released, then adds the key to the inputted code
def on_release(key):
    key_input.append(key)
    
    
#required function for the keyboard listner program to work
def on_press(key):
    pass
        

#changes the servo position to block the trigger
def block_trigger():
    thread1 = True
    pwm.ChangeDutyCycle(5) 
    sleep(0.5)
    pwm.ChangeDutyCycle(0)
    thread2 = False


#changes the servo position to unblock the trigger
def unblock_trigger():
    thread2 = True
    pwm.ChangeDutyCycle(3)
    sleep(0.5)
    pwm.ChangeDutyCycle(0)
    trigger_blocked = False
    thread2 = False
    
    
 
#find path of file containing data of faces, upperpodies, lowerbodies, and fullbodies
cascPathface = os.path.dirname(cv2.__file__) + "/data/haarcascade_frontalface_alt2.xml"
cascPathfull = os.path.dirname(cv2.__file__) + "/data/haarcascade_fullbody.xml"
cascPathupper = os.path.dirname(cv2.__file__) + "/data/haarcascade_upperbody.xml"
cascPathlower = os.path.dirname(cv2.__file__) + "/data/haarcascade_lowerbody.xml"

#load the data in the algorithm
faceCascade = cv2.CascadeClassifier(cascPathface)
bodyCascade = cv2.CascadeClassifier(cascPathfull)
upperCascade = cv2.CascadeClassifier(cascPathupper)
lowerCascade = cv2.CascadeClassifier(cascPathlower)

#starts the video stream
video_capture = cv2.VideoCapture(0)

#starts the keyboard listner
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()


while True:
    #grab the frame from the video stream and turn it into a format that the algorithm can compute
    ret, frame = video_capture.read()
    frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #runs the frame grabbed through the detection algrorithm that was collected earlier
    bodies = bodyCascade.detectMultiScale(gray,
                                          scaleFactor=1.1,
                                          minNeighbors=5,
                                          minSize=(60, 60),
                                          flags=cv2.CASCADE_SCALE_IMAGE)
    faces = faceCascade.detectMultiScale(gray,
                                         scaleFactor=1.1,
                                         minNeighbors=5,
                                         minSize=(60, 60),
                                         flags=cv2.CASCADE_SCALE_IMAGE)
    uppers = upperCascade.detectMultiScale(gray,
                                          scaleFactor=1.1,
                                          minNeighbors=5,
                                          minSize=(60, 60),
                                          flags=cv2.CASCADE_SCALE_IMAGE)
    lowers = lowerCascade.detectMultiScale(gray,
                                          scaleFactor=1.1,
                                          minNeighbors=5,
                                          minSize=(60, 60),
                                          flags=cv2.CASCADE_SCALE_IMAGE)
 
 
    #check if there is any data in the lists, (if there is no data then it does not detect a person)
    if len(faces) > 0 or len(uppers) > 0 or len(bodies) > 0 or len(lowers) > 0:
        if trigger_blocked == False and thread1 == False and thread2 == False:
            #starts a thread that allows the program to block the trigger while still being able to grab and analyze frames
            t1 = Thread(target = block_trigger)
            t1.start()
            trigger_blocked = True
            
    #check if the trigger is blocked and no threads are being ran
    elif trigger_blocked and thread1 == False and thread2 == False:
        t2 = Thread(target = unblock_trigger)
        t2.start()
        trigger_blocked = False
    
    #checks if the keypad input matches the code, then changes the servo to unblock the trigger
    if key_input == code:
        trap = True
        key_input = []
        pwm.ChangeDutyCycle(3)
        sleep(0.5)
        pwm.ChangeDutyCycle(0)
        trigger_blocked = False
        #keeps the code in a loop so it cannot preform face detection until the code is inputted again 
        while trap:
            if key_input == code:
                trap = False
                key_input = []
            if len(key_input) == 4 and key_input != code:
                key_input = []
                print('WRONG PASSCODE')
            
    #if the wrong code is inputted it resets the value for the inputted code
    if len(key_input) == 4 and key_input != code:
        key_input = []
        print('WRONG PASSCODE')


    #finds the position of the faces
    for (x, y, w, h) in faces:
        #draws a rectangle around the faces
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    
    for (x, y, w, h) in bodies:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
    for (x, y, w, h) in uppers:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    for (x, y, w, h) in lowers:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
    
    #displays the frame in a window
    cv2.imshow("Frame", frame)
    
    #checks for the 'q' key, then quits if it is pressed
    if cv2.waitKey(1) == ord('q'):
        break
   
#closes the window and stops the communication line between the servo motor
video_capture.release()
cv2.destroyAllWindows()
pwm.stop()