# import the necessary packages p
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt

#GPIO setup
GPIO.setmode(GPIO.BCM)
STEP=12
DIR=20
UP=1
DOWN=0
M0=17
M1=27
M2=22
GPIO.setup(STEP,GPIO.OUT) #STEP, DIR and Mx ports for interfacing with the DRV8825 
GPIO.setup(DIR,GPIO.OUT)
GPIO.setup(M0,GPIO.OUT)
GPIO.setup(M1,GPIO.OUT)
GPIO.setup(M2,GPIO.OUT)

#Variable definitions
global fs #frequency of PWM signal
fs=1
lastok=1 #to fix problem on startup to do with line 277
error=0

#initialize the PWM signal
PWM=GPIO.PWM(STEP,fs)
PWM.start(0)

#function definitions

#Step mode functions
def fullstep():
	GPIO.output(M0,0)
	GPIO.output(M1,0)
	GPIO.output(M2,0)
	fs=1000
	return fs

def halfstep():
        GPIO.output(M0,1)
        GPIO.output(M1,0)
        GPIO.output(M2,0)
        fs=3000
        return fs

def quarterstep():
        GPIO.output(M0,0)
        GPIO.output(M1,1)
        GPIO.output(M2,0)
        fs=11000 
        return fs

def eighthstep():
        GPIO.output(M0,1)
        GPIO.output(M1,1)
        GPIO.output(M2,0)
        fs=260000 
        return fs
    
def sixteenthstep():
        GPIO.output(M0,0)
        GPIO.output(M1,0)
        GPIO.output(M2,1)
        fs=240000 
        return fs

def microstep():
        GPIO.output(M0,1)
        GPIO.output(M1,0)
        GPIO.output(M2,1)
        fs=250000 
        return fs

def choosestep(e):
        if(9<=e): fs=quarterstep()  
        elif(5<e<9): 
            fs=eighthstep()
        elif(2<e<=5): 
            fs=sixteenthstep()
        else:
            fs=microstep()
        freq_array.append(fs/1000)
        
        if((len(freq_array)<2) or(freq_array[-1]!=freq_array[-2])):
            PWM.ChangeFrequency(fs)
            PWM.ChangeDutyCycle(50)


#function for generating plots and performance parameters at the end of each test        
def endplot(time_ar,freq_ar,error_ar):
    es_array=[] #eighth step array
    ms_array=[] #micro step array
    ss_array=[] #sixteenth step array
    qs_array=[] #quarter step array
    
    #fetches the frequencies of each step function before cleaning up the GPIO ports
    fs_es=eighthstep()/1000
    fs_ms=microstep()/1000
    fs_ss=sixteenthstep()/1000
    fs_qs=quarterstep()/1000
    GPIO.cleanup()
    
    for j in range(len(time_ar)):
            
            #lists are generated for the instances in which each step mode is activated

            if(abs(freq_ar[j])==fs_es): #eighth step
                es_array.append(freq_array[j]/abs(freq_array[j]))
                ms_array.append(0)
                ss_array.append(0)
                qs_array.append(0)
            
            elif(abs(freq_ar[j])==fs_ss): #sizteenth step
                ss_array.append(freq_array[j]/abs(freq_array[j]))
                es_array.append(0)
                ms_array.append(0)
                qs_array.append(0)
            
            elif (abs(freq_ar[j])==fs_ms): #micro step
                ms_array.append(freq_array[j]/abs(freq_array[j]))
                ss_array.append(0)
                es_array.append(0)
                qs_array.append(0)

            elif (abs(freq_ar[j])==fs_qs): #quarer step
                qs_array.append(freq_array[j]/abs(freq_array[j]))
                ss_array.append(0)
                es_array.append(0)
                ms_array.append(0)
            
            else:
                ms_array.append(0)
                ss_array.append(0)
                es_array.append(0)
                qs_array.append(0)

    
    for i in range(len(time_ar)):
        ms_array[i]=error_ar[i]*ms_array[i]
        ss_array[i]=error_ar[i]*ss_array[i]
        es_array[i]=error_ar[i]*es_array[i]
        qs_array[i]=error_ar[i]*qs_array[i]
    
    #graph plotting    
    fig, ax1=plt.subplots()
    
    color='tab:red'
    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('Error [pixels]')

    ax1.plot(time_ar,ms_array,color='green', label='micro step')
    ax1.fill_between(time_ar,ms_array,y2=0,color='green')
    
    ax1.plot(time_ar,ss_array,color='orange',label='sixteenth step')
    ax1.fill_between(time_ar,ss_array,y2=0,color='orange')
    
    ax1.plot(time_ar,es_array,color=color, label='eighth step')
    ax1.fill_between(time_ar,es_array,y2=0,color=color)
    
    ax1.plot(time_ar,qs_array,color='purple', label='quarter step')
    ax1.fill_between(time_ar,qs_array,y2=0,color='purple')
    
    ax1.legend()
    ax1.tick_params(axis='y',labelcolor='black')
    
    ax2=ax1.twinx()
    
    color='tab:blue'

    ax2.plot(time_ar,error_ar,color=color)
    ax2.plot(time_ar,np.zeros(len(time_ar)),'b--')
   
    #calculation of each performance metric for a run

    for i in range (len(error_ar)):
        error_ar[i]=abs(error_ar[i])
    
    IAE=np.trapz(error_ar,time_ar)
    print("IAE= "+ str(IAE))
    
    error_ar0=[]
    for i in range(len(error_ar)):
        error_ar0.append(time_ar[i]*error_ar[i])
    ITAE=np.trapz(error_ar0,time_ar)
    print("ITAE= "+ str(ITAE))
    
    for i in range(len(error_ar)):
        error_ar[i]=error_ar[i]**2
    MSE=sum(error_ar)/len(error_ar)
    print("MSE= "+ str(MSE))
    
    ISE=np.trapz(error_ar,time_ar)
    print("ISE= "+ str(ISE))
    
    fig.tight_layout()
    plt.savefig('graph.png') #save each graph generated as a PNG file
    plt.show()

#function to perform object detection using Haar-cascades

#https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def look(image):
    x=1
    y=1
    w=1
    h=1

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            #returns positions of detected faces in Rect(x,y,w,h)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            #finding faces, their sizes, rectangles (gives coordinates of roi)
    for (x,y,w,h) in faces:
             #rectangle(inputarray, vertex of rectangle, opposite vertex, color, thickness)
        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2) #(x+int(w/2),y+int(h/2)) is centre 
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = image[y:y+h, x:x+w]
            #cv2.imshow('img',image)
    error=0   
    return(x,y,w,h)
    
# initialize the camera
camera = PiCamera()
camera.rotation=180 
camera.resolution = (160, 128) 
camera.framerate = 60 
camera.video_stabilization=True
camera.color_effects=(128,128) #convert image to black and white
time.sleep(0.5) #delay for camera boot-up time

#start recording video
out = cv2.VideoWriter('video_recording_0.avi',cv2.VideoWriter_fourcc(*'DIVX'), 20, (160,128))

#create array of errors to plot at the end
global error_array
error_array=[]
time_start=time.time()
global error_timer
error_timer=[]
global freq_array
freq_array=[]

#grab an initial reference for raw data capture from the camera
rawCapture = PiRGBArray(camera, size=(160, 128))
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

#initialize the MFT tracking algorithm
tracker=cv2.TrackerMedianFlow_create() 

#generate an initial bounding box using the look function
rawCapture.truncate(0)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image=frame.array
    (x,y,w,h)=look(image)
    rawCapture.truncate(0)
    if(x!=1):break
    
bbox=(int(x+w/4),int(y+h/4),int(w/2),int(h/2)) 
w0=bbox[2] #w0 and h0 are used to fix a glitch where the bounding box grows from camera jitters
h0=bbox[3]
image = rawCapture.array 
ok=tracker.init(image,bbox) 

rawCapture.truncate(0)

# capture frames from the camera to generate a video feed
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	image = frame.array #current image frame

	#start timer, used for plots at the end and to get FPS
	timer=cv2.getTickCount()
	
	#update tracker
	ok,bbox=tracker.update(image)
	
	#calculate FPS
	fps=cv2.getTickFrequency() / (cv2.getTickCount() - timer);

	AreaRatio=(bbox[3]*bbox[2]/(w0*h0)) #used to fix bounding box enlargement problem
	        
	if ((ok)and(x!=1)and(AreaRatio<=2.1)):

        # Tracking success
		p1 = (int(bbox[0]), int(bbox[1]))
		p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))

        #draw bounding box and centre circle on image using opencv
		cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)
		cv2.circle(image, (int(bbox[0]+bbox[2]/2),int(bbox[1]+bbox[3]/2)),4,(0,0,255))

		error=bbox[1]+(bbox[3]/2)-64 #tracking error
		error_array.append(error) #add error to array for plotting
		error_timer.append(time.time()-time_start) #add current time to array for plotting
		
        #turn off motor if error small enough
		if(-0.5<(error)<0.5): 
			PWM.ChangeDutyCycle(0)
			freq_array.append(0)

		elif((error)<0): #use the sign of the current error to determine the required direction of vertical motion
			GPIO.output(DIR,UP) #set the DIR pin of the motor driver to the correct direction
			choosestep(abs(error)) #use the choosestep function to set the PWM signal and determine the step mode from the error
			
		else:
			GPIO.output(DIR,DOWN)
			choosestep(abs(error))
		
		lastok=cv2.getTickCount() #used for calculating delay to reinitialize tracker with detection algorithm

	else :
        # Tracking failure
		PWM.ChangeDutyCycle(0)
		cv2.putText(image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
		freq_array.append(0)
		error_array.append(0) #add error to array for plotting
		error_timer.append(time.time()-time_start)
		
		if((AreaRatio>1.5)or(0.5<((1/cv2.getTickFrequency())*(cv2.getTickCount()-lastok)))): #time where lost face for too long (more than 0.5 s seconds)
                    print((cv2.getTickCount()-lastok)/cv2.getTickFrequency())
                    tracker=cv2.TrackerMedianFlow_create()

                    #using look to reinitialize tracking algorithm
                    x=1
                    (x,y,w,h)=look(image) #x=1 if no faces found. 
                    if(x!=1):
                        bbox=(int(x+w/4),int(y+h/4),int(w/2),int(h/2))
                        w0=bbox[2]
                        h0=bbox[3]
                        image = rawCapture.array
                        ok=tracker.init(image,bbox)
                
        # Display tracker type on image frame
	cv2.putText(image, "MFT Tracker", (70,20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (50,170,50),1);

        # Display FPS on image frame
	cv2.putText(image, "FPS : " + str(int(fps)), (70,30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (50,170,50), 1);
	cv2.putText(image, "Error : " +format(error,'.3f'), (70,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255),1);

        # Display image
	cv2.imshow("Tracking", image)
	out.write(image)

	key = cv2.waitKey(1) & 0xFF #get input from keyboard
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# end program when q is called
	if key == ord("q"):

		out.release() #for video recording to stop
		endplot(error_timer,freq_array,error_array) #use endplot to generate graphs and performance metrics
		break
