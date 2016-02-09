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

PWM_PIN_1 = 16
PWM_PIN_2 = 21

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(PWM_PIN_1, GPIO.OUT)
# GPIO.setup(PWM_PIN_2, GPIO.OUT)

# pwm1 = GPIO.PWM(PWM_PIN_1, 320)
# pwm2 = GPIO.PWM(PWM_PIN_2, 320)
# pwm1.start(1)
# pwm2.start(1)
 
# use BCM GPIO numbers                                                        
wiringpi.wiringPiSetupGpio()

pwm_pin_horizontal = 18
pwm_pin_vertical = 13

pwm_clock_1 = 400
pwm_clock_2 = 400

pwm_range = 1024

dt_min = 50
dt_max = 95

dt_horizontal = dt_min
dt_vertical = dt_min

# enable PWM0                                                                 
wiringpi.pinMode(pwm_pin_horizontal, 2)
wiringpi.pwmSetMode(0)
wiringpi.pwmSetClock(pwm_clock_1)
wiringpi.pwmSetRange(pwm_range)

# enable PWM1                                                                 
wiringpi.pinMode(pwm_pin_vertical, 2)
wiringpi.pwmSetMode(0)
wiringpi.pwmSetClock(pwm_clock_2)
wiringpi.pwmSetRange(pwm_range)

time_start = time.time()
cycle = 5		

wiringpi.pwmWrite(pwm_pin_horizontal, 72)
wiringpi.pwmWrite(pwm_pin_vertical, 72)

time.sleep(1)

def transmitMoveOrder(horizontal, vertical):
	if horizontal != 0:
		global dt_horizontal
		dt_horizontal += horizontal
		if dt_horizontal > dt_max:
			dt_horizontal = dt_max
		if dt_horizontal < dt_min:
			dt_horizontal = dt_min

		wiringpi.pwmWrite(pwm_pin_horizontal, int(dt_horizontal))
	if vertical != 0:
		global dt_vertical
		dt_vertical += vertical
		if dt_vertical > dt_max:
			dt_vertical = dt_max
		if dt_vertical < dt_min:
			dt_vertical = dt_min

		wiringpi.pwmWrite(pwm_pin_vertical, int(dt_vertical))
	time.sleep(0.01)
	return 1
 
def checkPosition(posX, posY, maxX, maxY):
	centerX = maxX / 2
	centerY = maxY / 2

	# distance of object from the center of screen
	distanceX = abs(posX - centerX);
	distanceY = abs(posY - centerY);	

	maxDistanceX = maxX / 2
	maxDistanceY = maxY / 2

	# required distance from center to object (in percent of screen) to send move order
	threshold = 8 # percent
	maxSpeed = 600
	minSpeed = 100
	speedRange = maxSpeed - minSpeed

	# if distance does not hit threshold leave it to zero
	moveX = 0.0
	moveY = 0.0
	
	relativePosX = posX - centerX # negative if left
	relativePosY = posY - centerY # negative if up
	
	maxMove = 10.0
	if distanceX > maxX * (0.01 * threshold):
		if relativePosX < 0:	# move left
			moveX = -1 * (distanceX / maxDistanceX) * maxMove
			# print("LEFT")
		else:
			moveX =  (distanceX / maxDistanceX) * maxMove
			# print("RIGHT")
	if distanceY > maxY * (0.01 * threshold):
		if relativePosY < 0:	# move up
			# print("UP")
			moveY = -1 * (distanceY / maxDistanceY) * maxMove
		else:				# move down
			moveY =  (distanceY / maxDistanceY) * maxMove					
			# print("DOWN")
	if moveX != 0 or moveY != 0:
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
# and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=0).start()
# fps = FPS().start()

#Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier('facecascade.xml')
#start_time = time.time()

frame = vs.read()
imageHeight, imageWidth = frame.shape[:2]

# loop over some frames...this time using the threaded stream
while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()

 	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #Look for faces in the image using the loaded cascade file
	faces = face_cascade.detectMultiScale(gray, 1.8, 5)

	print ("Found ", len(faces))
	
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
		#elapsed_time = time.time() - start_time

		frame = imutils.resize(frame, width=400)
		# if elapsed_time > 2:
		#start_time = time.time()		
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

 	# cv2.imwrite('result.jpg', frame)
	# update the FPS counter
	# fps.update()
 	# print("--- frame processed ---")

# stop the timer and display FPS information
# fps.stop()
#print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
#print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
