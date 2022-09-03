from picamera import PiCamera
import time
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import cv2

# Initialize face detector for find_faces
# https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')


def initialize_camera():

    camera = PiCamera()
    camera.rotation = 180
    camera.resolution = (160, 128)
    camera.framerate = 60
    camera.video_stabilization = True
    # convert image to black and white
    camera.color_effects = (128, 128)
    # delay for camera boot-up time
    time.sleep(0.5)

    return camera


def find_faces(image):

    # Initialization for case where no faces found
    x = 0
    y = 0
    w = 0
    h = 0

    # Grayscale of input image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Returns positions of detected faces in Rect(x,y,w,h)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    # Finding faces, their sizes, rectangles (gives coordinates of roi)
    for (x, y, w, h) in faces:

        # Plot rectangle around face
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    return (x, y, w, h)


# function for generating plots and performance parameters at the end of
# each test
def endplot(time_ar, freq_ar, error_ar, controller):

    es_array = []  # eighth step array
    ms_array = []  # micro step array
    ss_array = []  # sixteenth step array
    qs_array = []  # quarter step array

    # fetches the frequencies of each step function before cleaning up the
    # GPIO ports
    fs_es = controller.eighthstep() / 1000
    fs_ms = controller.microstep() / 1000
    fs_ss = controller.sixteenthstep() / 1000
    fs_qs = controller.quarterstep() / 1000
    GPIO.cleanup()

    for j in range(len(time_ar)):

        # Masks are generated for the instances in which each step mode is
        # activated

        if (abs(freq_ar[j]) == fs_es):  # eighth step
            es_array.append(
                controller.freq_array[j] / abs(controller.freq_array[j]))
            ms_array.append(0)
            ss_array.append(0)
            qs_array.append(0)

        elif (abs(freq_ar[j]) == fs_ss):  # sixteenth step
            ss_array.append(
                controller.freq_array[j] / abs(controller.freq_array[j]))
            es_array.append(0)
            ms_array.append(0)
            qs_array.append(0)

        elif (abs(freq_ar[j]) == fs_ms):  # micro step
            ms_array.append(
                controller.freq_array[j] / abs(controller.freq_array[j]))
            ss_array.append(0)
            es_array.append(0)
            qs_array.append(0)

        elif (abs(freq_ar[j]) == fs_qs):  # quarer step
            qs_array.append(
                controller.freq_array[j] / abs(controller.freq_array[j]))
            ss_array.append(0)
            es_array.append(0)
            ms_array.append(0)

        else:
            ms_array.append(0)
            ss_array.append(0)
            es_array.append(0)
            qs_array.append(0)

    for i in range(len(time_ar)):
        ms_array[i] = error_ar[i] * ms_array[i]
        ss_array[i] = error_ar[i] * ss_array[i]
        es_array[i] = error_ar[i] * es_array[i]
        qs_array[i] = error_ar[i] * qs_array[i]

    # Graph plotting
    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('Error [pixels]')

    ax1.plot(time_ar, ms_array, color='green', label='micro step')
    ax1.fill_between(time_ar, ms_array, y2=0, color='green')

    ax1.plot(time_ar, ss_array, color='orange', label='sixteenth step')
    ax1.fill_between(time_ar, ss_array, y2=0, color='orange')

    ax1.plot(time_ar, es_array, color=color, label='eighth step')
    ax1.fill_between(time_ar, es_array, y2=0, color=color)

    ax1.plot(time_ar, qs_array, color='purple', label='quarter step')
    ax1.fill_between(time_ar, qs_array, y2=0, color='purple')

    ax1.legend()
    ax1.tick_params(axis='y', labelcolor='black')

    ax2 = ax1.twinx()

    color = 'tab:blue'

    ax2.plot(time_ar, error_ar, color=color)
    ax2.plot(time_ar, np.zeros(len(time_ar)), 'b--')

    # Calculation of each performance metric for a run
    for i in range(len(error_ar)):
        error_ar[i] = abs(error_ar[i])

    IAE = np.trapz(error_ar, time_ar)
    print("IAE= " + str(IAE))

    error_ar0 = []
    for i in range(len(error_ar)):
        error_ar0.append(time_ar[i] * error_ar[i])
    ITAE = np.trapz(error_ar0, time_ar)
    print("ITAE= " + str(ITAE))

    for i in range(len(error_ar)):
        error_ar[i] = error_ar[i]**2
    MSE = sum(error_ar) / len(error_ar)
    print("MSE= " + str(MSE))

    ISE = np.trapz(error_ar, time_ar)
    print("ISE= " + str(ISE))

    fig.tight_layout()
    plt.savefig('graph.png')  # save each graph generated as a PNG file
    plt.show()
