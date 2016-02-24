
# import the necessary packages
from __future__ import print_function
from __future__ import division
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import math
import argparse
import imutils
import cv2
import time
import RPi.GPIO as GPIO
import wiringpi2 as wiringpi
import time

# use BCM GPIO numbers
wiringpi.wiringPiSetupGpio()

pwm_pin_horizontal = 18
pwm_pin_vertical = 13

pwm_clock = 200
pwm_range = 1024

dt_min = 125
dt_max = 175

dt_horizontal = 150
dt_vertical = 150

# changing PWM

verticalPWM = 0
verticalPWMSpeed = 0.5

# enable PWM0
wiringpi.pinMode(pwm_pin_horizontal, 2)
wiringpi.pwmSetMode(0)
wiringpi.pwmSetClock(pwm_clock)
wiringpi.pwmSetRange(pwm_range)

# enable PWM1
wiringpi.pinMode(pwm_pin_vertical, 2)
wiringpi.pwmSetMode(0)
wiringpi.pwmSetClock(pwm_clock)
wiringpi.pwmSetRange(pwm_range)

#led
faceFoundPin = 5
wiringpi.pinMode(faceFoundPin, 1)

faceLost = True
faceLostStart = -1
faceLostPwm = -1
faceLostTimeout = 5

time_start = time.time()

wiringpi.pwmWrite(pwm_pin_horizontal, dt_horizontal)
wiringpi.pwmWrite(pwm_pin_vertical, dt_vertical)

time.sleep(0.2)

def transmitMoveOrder(horizontal, vertical):
    global dt_horizontal
    dt_horizontal = 150 + horizontal
    if dt_horizontal > dt_max:
            dt_horizontal = dt_max
    if dt_horizontal < dt_min:
            dt_horizontal = dt_min
    wiringpi.pwmWrite(pwm_pin_horizontal, int(dt_horizontal))

    global dt_vertical
    dt_vertical = 150 + vertical
    if dt_vertical > dt_max:
            dt_vertical = dt_max
    if dt_vertical < dt_min:
            dt_vertical = dt_min
    wiringpi.pwmWrite(pwm_pin_vertical, int(dt_vertical))

    print("Horizontal: ",dt_horizontal)
    print("Vertical: ",dt_vertical)
    time.sleep(0.01)
    return 1

def checkPosition(posX, posY, maxX, maxY):
    global verticalPWM
    global verticalPWMSpeed
    global faceLostPwm

    centerX = maxX / 2
    centerY = maxY / 2

    # distance of object from the center of screen
    distanceX = abs(posX - centerX);
    distanceY = abs(posY - centerY);

    maxDistanceX = maxX / 2
    maxDistanceY = maxY / 2

    # required distance from center to object (in percent of screen) to send move order
    thresholdX = 20 # percent
    thresholdY = 12

    # if distance does not hit threshold leave it to zero
    moveX = 0.0
    moveY = 0.0

    relativePosX = posX - centerX # negative if left
    relativePosY = posY - centerY # negative if up

    maxMoveX = 3.6
    #maxMoveY = 18.0

    if distanceX > maxX * (0.01 * thresholdX):
        if relativePosX < 0:    # move left
            # relative speed
            # XmoveX = -1 * (distanceX / maxDistanceX) * maxMoveX

            # static speed
            moveX = -1 * maxMoveX

            #print("LEFT")
        else:
            # relative speed
            # moveX =  maxMoveX * (distanceX / maxDistanceX) * maxMoveX

            # static speed
            moveX = maxMoveX

            # print("RIGHT")
    if distanceY > maxY * (0.01 * thresholdY):
        if relativePosY < 0:    # move up
            print("UPUPUP")
            # changing PWM
            verticalPWM = verticalPWM - verticalPWMSpeed
            #moveY = verticalPWM
            # relative speed
            # moveY = -1 * (distanceY / maxDistanceY) * maxMoveY

            # static speed
            #moveY = -1 * maxMoveY
        else:                   # move down
            # changing PWM
            verticalPWM = verticalPWM + verticalPWMSpeed
            #moveY = verticalPWM
            # relative speed
            #moveY = (distanceY / maxDistanceY) * maxMoveY

            # static speed
            #moveY = maxMoveY
            # print("DOWN")
    print("VERTICAL_PWM ", verticalPWM)
    moveY = verticalPWM
    faceLostPwm = moveY
    transmitMoveOrder(moveX, moveY);

    return 1

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
        help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
        help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

# created a *threaded* video stream, allow the camera sensor to warmup,
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=0).start()

#Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier('facecascade.xml')
start_time = time.time()

frame = vs.read()
imageHeight, imageWidth = frame.shape[:2]

fpsCounter = 0
# loop over some frames...this time using the threaded stream

try:
    while True:
        currTime = time.time()
        if currTime - start_time >= 1:
            print("FPS: ", fpsCounter)
            fpsCounter = 0
            start_time = currTime
        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 400 pixels
        frame = vs.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #Look for faces in the image using the loaded cascade file
        faces = face_cascade.detectMultiScale(gray, 1.8, 3)

        print ("Found ", len(faces))

        if len(faces) == 0:
            if faceLost == False:
                faceLostStart = time.time()
                faceLost = True
                print("--------- FACE LOST ---------")
            else:
                if time.time() - faceLostStart > faceLostTimeout:
                    faceLostPwm = 0
                    verticalPWM = 0
                #print("DIGITAL WRITE 0")
                wiringpi.digitalWrite(faceFoundPin, 0)
                transmitMoveOrder(0, faceLostPwm)
        else:
            if faceLost == True:
                faceLost = False
                faceLostStart = -1
            #print("DIGITAL WRITE 1")
            wiringpi.digitalWrite(faceFoundPin, 1)

        firstFace = True
        #Draw a rectangle around every found face
        for (x, y, w, h) in faces:
            if firstFace:
                cv2.rectangle(frame,(x, y), (x + w, y + h), (0, 255, 0), 2)
                checkPosition(x + w / 2, y + h / 2, imageWidth, imageHeight)
                firstFace = False
            else:
                cv2.rectangle(frame,(x, y), (x + w, y + h), (0, 0, 255), 2)

        # check to see if the frame should be displayed to our screen
        if args["display"] > 0:
            frame = imutils.resize(frame, width=400)

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

        fpsCounter = fpsCounter + 1
        # print("--- frame processed ---")

except KeyboardInterrupt:
        # here you put any code you want to run before the program
        # exits when you press CTRL+C
        print("Keyboard interrupt")
except:
        # this catches ALL other exceptions including errors.
        # You won't get any error messages for debugging
        # so only use it once your code is working
        print("Other error or exception occurred!")
finally:
        print("Cleaning up...")
        wiringpi.pinMode(pwm_pin_vertical, 0)
        wiringpi.pinMode(pwm_pin_horizontal, 0)
        wiringpi.pinMode(faceFoundPin, 0)
        # GPIO.cleanup() # this ensures a clean exit
        cv2.destroyAllWindows()
        vs.stop()

