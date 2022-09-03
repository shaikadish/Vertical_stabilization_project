import cv2

class Visualizer():
    def __init__(self) -> None:
        pass

    def show_box(image,bbox):
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))

        # Draw bounding box and centre circle on image using opencv
        cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)
        cv2.circle(image, (int(bbox[0]+bbox[2]/2),int(bbox[1]+bbox[3]/2)),4,(0,0,255))

    def show_failure(image):
        cv2.putText(image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
    def show_output(image,fps,error):
        # Display tracker type on image frame
        cv2.putText(image, "MFT Tracker", (70,20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (50,170,50),1);

        # Display FPS on image frame
        cv2.putText(image, "FPS : " + str(int(fps)), (70,30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (50,170,50), 1);
        cv2.putText(image, "Error : " +format(error,'.3f'), (70,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255),1);

        # Display image
        cv2.imshow("Tracking", image)