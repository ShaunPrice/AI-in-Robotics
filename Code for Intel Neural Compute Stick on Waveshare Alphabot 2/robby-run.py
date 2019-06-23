#!/usr/bin/env python3

from picamera import PiCamera
import picamera.array
import io
import numpy as np
import time
import cv2 as cv
import glob
import RPi.GPIO as GPIO
from AlphaBot import AlphaBot
from PCA9685 import PCA9685

'''
Notes:
	1. Servos are from 500 to 2500 with 1500 centred
'''

speed = 40
turn_speed = 100

# Function to capture the image and return the NCS result
def getPathState():
	with picamera.array.PiRGBArray(camera) as stream:
		camera.capture(stream, format="bgr")
		image = stream.array

	# Crop  to square and scale to 224x224
	y,x,z = image.shape
	x = (x-y)//2
	image = image[0:y, x:x+y]

	dim = (int(224), int(224))
	image = cv.resize(image, dim, interpolation = cv.INTER_AREA)
	#cv.imshow("Camera",image)
	#cv.waitKey(0)

	# Prepare input blob and perform an inference.
	blob = cv.dnn.blobFromImage(image, size=(224, 224), ddepth=cv.CV_8U)
	net.setInput(blob)
	return net.forward()

# Configure the motors and servos
robot = AlphaBot()
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

#Set the Horizontal servo parameters
HPulse = 1500  #Sets the initial Pulse
HStep = 0      #Sets the initial step length
pwm.setServoPulse(0,HPulse)

#Set the vertical servo parameters
VPulse = 1350  #Sets the initial Pulse
VStep = 0      #Sets the initial step length
pwm.setServoPulse(1,VPulse)

robot.setPWMA(float(speed))
robot.setPWMB(float(speed))


# start the camera
camera = PiCamera()
camera.rotation = 180
stream = io.BytesIO()

# Warm-up camera
time.sleep(2)

# Load the model.
net = cv.dnn.readNet('image-classification-0002.xml', 'image-classification-0002.bin')

# Specify target device.
net.setPreferableTarget(cv.dnn.DNN_TARGET_MYRIAD)

# Loop until CTRL+C
while True:
    # Look at determin if clear or not
	result = getPathState()

	for detection in result:
		if detection[0] > detection[1]:
			print('clear:  \t'+str(detection[0]))
			# Look forward
			HPulse = 1500
			#VPulse = 1500
			pwm.setServoPulse(0,HPulse)
			#pwm.setServoPulse(1,VPulse)
			time.sleep(0.25)
			# Move forward
			robot.setPWMA(float(speed))
			robot.setPWMB(float(speed))
			robot.forward()
		else:
			print('blocked:\t'+str(detection[1]))
			# stop
			robot.stop()
			
			# look left
			HPulse = 800
			pwm.setServoPulse(0,HPulse)
			time.sleep(0.25)
			
			# Check clear/blocked
			left_result = getPathState()
			
			# look right
			HPulse = 2200
			pwm.setServoPulse(0,HPulse)
			time.sleep(0.25)
			
			# Check clear/blocked
			right_result = getPathState()
			
			# Set teh speed to turn
			robot.setPWMA(float(turn_speed))
			robot.setPWMB(float(turn_speed))

			# Which ever way is the most clear, turn that way
			left_clear = 0
			right_clear = 0
			
			if (left_result[0][0] > left_result[0][1]):
				left_clear = left_result[0][0]
			
			if (right_result[0][0] > right_result[0][1]):
    				right_clear = right_result[0][0]

			if (left_clear > right_clear):
				print("Turning Left")
				# Turn left and stop
				robot.left()
				time.sleep(0.8)
				robot.stop()
			else:
				print("Turning Right")
    			# Turn right and stop
				robot.right()
				time.sleep(0.8)
				robot.stop()

			# Look forward
			HPulse = 800
			pwm.setServoPulse(0,HPulse)
			time.sleep(0.4)

import RPi.GPIO as GPIO
import time

class AlphaBot(object):
	
	def __init__(self,in1=13,in2=12,ena=6,in3=21,in4=20,enb=26):
		self.IN1 = in1
		self.IN2 = in2
		self.IN3 = in3
		self.IN4 = in4
		self.ENA = ena
		self.ENB = enb
		self.PA  = 50
		self.PB  = 50

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.IN1,GPIO.OUT)
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		self.PWMA = GPIO.PWM(self.ENA,500)
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(self.PA)
		self.PWMB.start(self.PB)
		self.stop()

	def forward(self):
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

	def stop(self):
		self.PWMA.ChangeDutyCycle(0)
		self.PWMB.ChangeDutyCycle(0)
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)

	def backward(self):
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)

	def left(self):
		self.PWMA.ChangeDutyCycle(30)
		self.PWMB.ChangeDutyCycle(30)
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

	def right(self):
		self.PWMA.ChangeDutyCycle(30)
		self.PWMB.ChangeDutyCycle(30)
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)
		
	def setPWMA(self,value):
		self.PA = value
		self.PWMA.ChangeDutyCycle(self.PA)

	def setPWMB(self,value):
		self.PB = value
		self.PWMB.ChangeDutyCycle(self.PB)	
		
	def setMotor(self, left, right):
		if((right >= 0) and (right <= 100)):
			GPIO.output(self.IN1,GPIO.HIGH)
			GPIO.output(self.IN2,GPIO.LOW)
			self.PWMA.ChangeDutyCycle(right)
		elif((right < 0) and (right >= -100)):
			GPIO.output(self.IN1,GPIO.LOW)
			GPIO.output(self.IN2,GPIO.HIGH)
			self.PWMA.ChangeDutyCycle(0 - right)
		if((left >= 0) and (left <= 100)):
			GPIO.output(self.IN3,GPIO.HIGH)
			GPIO.output(self.IN4,GPIO.LOW)
			self.PWMB.ChangeDutyCycle(left)
		elif((left < 0) and (left >= -100)):
			GPIO.output(self.IN3,GPIO.LOW)
			GPIO.output(self.IN4,GPIO.HIGH)
			self.PWMB.ChangeDutyCycle(0 - left)

if __name__=='__main__':

	Ab = AlphaBot()
	Ab.forward()
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		GPIO.cleanup()

#!/usr/bin/env python3

import cv2 as cv
import glob

# Load the model.
net = cv.dnn.readNet('image-classification-0002.xml', 'image-classification-0002.bin')

# Specify target device.
net.setPreferableTarget(cv.dnn.DNN_TARGET_MYRIAD)

# Read an image.
print('Loading blocked image')
frame = cv.imread('blocked.jpg')

# Get the list of images
images = glob.glob('images/*.jpg')

for image in images:
	# Read an image.
	print('Loading: '+image)
	frame = cv.imread(image)

	# Prepare input blob and perform an inference.
	blob = cv.dnn.blobFromImage(frame, size=(224, 224), ddepth=cv.CV_8U)
	net.setInput(blob)
	out = net.forward()

	for detection in out:
		print('clear:  \t'+str(detection[0]))
		print('blocked:\t'+str(detection[1]))
#!/usr/bin/python

import time
import math
import smbus

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)
	
  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
	  
  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result
	
  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))
	  
  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))

if __name__=='__main__':
 
  pwm = PCA9685(0x40, debug=True)
  pwm.setPWMFreq(50)
  while True:
   # setServoPulse(2,2500)
    for i in range(500,2500,10):  
      pwm.setServoPulse(0,i)   
      time.sleep(0.02)     
    
    for i in range(2500,500,-10):
      pwm.setServoPulse(0,i) 
      time.sleep(0.02)  
