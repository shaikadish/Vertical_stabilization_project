# import the necessary packages p
from picamera.array import PiRGBArray
import time
import cv2
from Controller import Controller
from gpioConfig import *
from utils import *
from Visualizer import Visualizer

# Initialize variables
lastok = 1
error = 0

# Initialize the camera
camera = initialize_camera()

# Initialize controller
controller = Controller()

# Initialize Visualization
visualizer = Visualizer()

# Start recording video
out = cv2.VideoWriter(
    'video_recording_0.avi', cv2.VideoWriter_fourcc(
        *'DIVX'), 20, (160, 128))

# Create array of errors to plot at the end
error_array = []
time_start = time.time()
error_timer = []

# Grab an initial reference for raw data capture from the camera
rawCapture = PiRGBArray(camera, size=(160, 128))
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

# Initialize the MFT tracking algorithm
tracker = cv2.TrackerMedianFlow_create()

# Generate an initial bounding box
rawCapture.truncate(0)
for frame in camera.capture_continuous(
        rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    (x, y, w, h) = find_faces(image)
    rawCapture.truncate(0)
    if (x != 1):
        break

bbox = (int(x + w / 4), int(y + h / 4), int(w / 2), int(h / 2))

# w0 and h0 are used to fix a problem where the bounding box grows from
# camera jitters
w0 = bbox[2]
h0 = bbox[3]

image = rawCapture.array
ok = tracker.init(image, bbox)

rawCapture.truncate(0)

# Capture frames from the camera to generate a video feed
for frame in camera.capture_continuous(
        rawCapture, format="bgr", use_video_port=True):

    # Current image frame
    image = frame.array

    # Start timer, used for plots at the end and to get FPS
    timer = cv2.getTickCount()

    # Update tracker
    tracker_ok, bbox = tracker.update(image)

    # Calculate FPS
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

    # Used to fix bounding box enlargement problem
    AreaRatio = (bbox[3] * bbox[2] / (w0 * h0))

    # If succesful tracking
    if ((tracker_ok) and (x != 0) and (AreaRatio <= 2.1)):

        visualizer.show_box(image, bbox)

        # Tracking error
        error = bbox[1] + (bbox[3] / 2) - 64
        error_array.append(error)
        error_timer.append(time.time() - time_start)

        # Change controller direction and speed
        controller.change_direction(error)
        controller.choosestep(abs(error))

        # Used for calculating delay to reinitialize tracker with detection
        # algorithm
        lastok = cv2.getTickCount()

    # Tracking failure
    else:

        # Turn off Motor
        controller.change_direction(0)

        visualizer.show_failure(image)

        # Add error to array for plotting
        error_array.append(0)
        error_timer.append(time.time() - time_start)

        # Reinitialize tracker after 0.5s or if area ratio too large
        if ((AreaRatio > 1.5) or (
                0.5 < ((1 / cv2.getTickFrequency()) * (cv2.getTickCount() - lastok)))):

        tracker = cv2.TrackerMedianFlow_create()

        # Reinitialize tracking algorithm
        (x, y, w, h) = find_faces(image)
        # x==0  if no faces found.
        if (x != 1):
            bbox = (int(x + w / 4), int(y + h / 4), int(w / 2), int(h / 2))
            w0 = bbox[2]
            h0 = bbox[3]
            image = rawCapture.array
            ok = tracker.init(image, bbox)

    # Show frame output and save image
    visualizer.show_output(image, fps, error)
    out.write(image)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # End program when q is called
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        # Stop recording video
        out.release()
        # Generate graphs and performance metrics
        endplot(error_timer, controller.freq_array, error_array)
        break
