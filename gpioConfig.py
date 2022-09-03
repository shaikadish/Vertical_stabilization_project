
import RPi.GPIO as GPIO

# GPIO setup
GPIO.setmode(GPIO.BCM)
STEP = 12
DIR = 20
UP = 1
DOWN = 0
M0 = 17
M1 = 27
M2 = 22

# STEP, DIR and Mx ports for interfacing with the DRV8825
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(M0, GPIO.OUT)
GPIO.setup(M1, GPIO.OUT)
GPIO.setup(M2, GPIO.OUT)

# initialize the PWM signal
fs = 1
PWM = GPIO.PWM(STEP, fs)
PWM.start(0)
