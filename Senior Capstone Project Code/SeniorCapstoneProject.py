# This code was written for a Senior Capstone Project as a team.
# The people who were also part of the team are Siyue Huang, Robert Kading,
# Ashley Wang, and Abdul-Rauf Yakubu.

# The goal of the project is to design a device for operating an autonomous vehicle
# to follow a leader vehicle at a certain distance. A RC car was used as a testbed for the
# prototype of the device and the device was designed based on the engineering design process.

# For the project, the Raspberry Pi 3 operates the code to drive the RC car. The RPi 3 is
# connected to a RPi 3 camera module and a PWM driver (Adafruit PCA9685). The camera was used to detect
# a green colored paper (the marker). The PWM driver was used to operate the drive and steering motors
# with a clean PWM signal. Triangle similarity was used to find the distance from the marker using the
# focal length of the camera, actual height of the marker, and the pixel height of the bounding box of the marker
# (with the assumption that the sides of the marker used as the height would always be vertical in the camera image).

import cv2 as cv
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import time

# Initialize the image pixel width and height
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Set up the RPi camera 
camera=PiCamera()
camera.resolution=(FRAME_WIDTH,FRAME_HEIGHT)
camera.framerate=90
rawCapture=PiRGBArray(camera,size=(FRAME_WIDTH,FRAME_HEIGHT))

# Working green hsv ranges (light green paper representing the non-reflective marker)
icol=(30,35,53,87,255,255)

# Set the HSV value ranges for detecting the color specified
lowHue = icol[0]
lowSat = icol[1]
lowVal = icol[2]
highHue = icol[3]
highSat = icol[4]
highVal = icol[5]

# Set up PWM module to send PWM signal to ESC
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)     # Set the PWM frequency
servo=266                # Set the steering servo to the neutral state PWM
drive=312                # Set the drive motor to the neutral state PWM
pwm.set_pwm(1,0,servo)

#This parameter shows if the PWM for the RC car car has been set to move forward.
#The RC car needs a special step for it to be controlled from going forwards to going backwards.
forward=False            

# Desired distance (in meters) and max speed (the 12-bit step that corresponds to the allowed max speed) from leader vehicle
desired=1.5
max_speed=320

Kp, Ki, Kd=1.5, 0, 0.01   # Initialize PID parameter
sum_e=0     # Sum of the error throughout the run
dir_e=0     # The change in the error
prev_e=0    # The previous error

# Set the GPIO pins on the RPi for a switch
GPIO.setmode(GPIO.BOARD)
GPIO.setup(38,GPIO.OUT)
GPIO.output(38,1)
GPIO.setup(40,GPIO.IN)

# Set the gamma correction parameter
gamma=3

# Don't start the run until the switch is flipped
while GPIO.input(40)==0:
	time.sleep(0.5)
print('Starting')
time.sleep(1)

dir_time=time.time()

# The for loop captures an image from the RPi camera each loop
for image in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):

	# Get image data into an array for OpenCV
	frame=image.array
	rawCapture.truncate(0) #Need to do this before getting new image.array

	# Convert the image into HSV color model.
	frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

	# Perform the gamma correction
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
	frame=cv.LUT(frame,table)

	# HSV values to define a colour range we want to create a mask from.
	colorLow = np.array([lowHue,lowSat,lowVal])
	colorHigh = np.array([highHue,highSat,highVal])
	mask = cv.inRange(frameHSV, colorLow, colorHigh)

	# Perform erosion and dilation to remove noise
	kernel=np.ones((5,5),np.uint8)
	mask = cv.morphologyEx(mask,cv.MORPH_OPEN,kernel)

        # Find the contours from the HSV ranges
	contours, hierarchy =cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
	contour_sizes = [(cv.contourArea(contour), contour) for contour in contours]

	# If there is any contour, calculate the distance from the marker
	if len(contours)!=0:
                # Find the biggest from all the contours
		biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

                # Draw the biggest contour on the image (for debugging purposes)
		cv.drawContours(frame, biggest_contour, -1, (0,255,0), 3)

		# Find the bounding box around the biggest contour
		x,y,w,h = cv.boundingRect(biggest_contour)
		
		# If the bounding box is smaller than a certain size, don't consider it as the marker
		if(w*h>600): 
                        # Draw the bounding box around the biggest contour (for debugging purposes)
			cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

			# Draw the center of the marker (for debugging purposes)
			cv.circle(frame,(x+int(w/2),y+int(h/2)),2,(0,0,255),2)
			
			# Compute the error (horizontal distance of the center of the marker from center of image)
			e=FRAME_WIDTH/2-(x+w/2)
			# Convert the error into duty cycle of the PWM for the steering motor
			duty_cycle=((6.5+e/(FRAME_WIDTH/2)*2)/100*4096)
			servo=round(duty_cycle)
			# Print error and duty cycle for debugging purposes
			print('Error {0:f}'.format(e))
			print("Change duty cycle to: {0}".format(duty_cycle))
		
			#380 pixels for 9.25 inch
			#focal length of the camera
			f=390.0*0.762/0.23495
			distance=(0.187*f)/h 
			print('Distance: {0:f}'.format(distance))

			# Use PID for the controller
			dist_e=distance-desired         # Compute the distance error
 			sum_e=sum_e+dist_e              # Compute the sum of distance errors
			pid=Kp*dist_e+Kd*dir_e+Ki*sum_e # Compute the PID value
			prev_time=dir_time      
			dir_time=time.time()
			dir_e=(dist_e-prev_e)/(dir_time-prev_time)
			prev_e=dist_e

                        # Compute the duty cycle for the PWM (allowing for deadband)
			if dist_e>0.1:           # When the distance error is positive, drive forward
				prev_drive=drive
				drive=320
				drive=318+round(pid)
				if(drive> max_speed):
					drive=max_speed
				forward=True
			elif dist_e<-0.1:        # When the distance error is negative, drive backward
				prev_drive=drive 

				# If the RC car was moving forward, perform a special action to make ESC
				# tell the drive motor to move back
				if forward:
					drive=270
					pwm.set_pwm(0,0,drive)
					time.sleep(0.1)
					drive=313
					pwm.set_pwm(0,0,drive)
					time.sleep(0.1)
					forward=False
				drive=285
			#When the distance error is in the deadband, stop the RC car
			else:
				prev_drive=drive
				drive=312
			
			#Set PWM values for the drive motor and servo motor
			pwm.set_pwm(0,0,drive)
			time.sleep(0.02)
			pwm.set_pwm(1,0,servo)
			print('Drive pulse: {0:d}'.format(drive))

        #If there are no contours (there are no pixel that fits the HSV range), stop the RC car at is current state
	else:
		drive=312
		pwm.set_pwm(0,0,drive)	
		pwm.set_pwm(1,0,servo)

        #If the switch is flipped to be off, stop the run		
	if GPIO.input(40)==0:
		break

#Set the drive motor and steering motor to be off
time.sleep(0.05)
pwm.set_pwm(0,0,0)
pwm.set_pwm(1,0,0)
time.sleep(0.05)
print('Ending run')
