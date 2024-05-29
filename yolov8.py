import cv2
import ultralytics
from ultralytics import YOLO
import numpy as np
import serial
import time
from libkinect2 import Kinect2
from libkinect2.utils import depth_map_to_image
serial_port = 'COM23'

ser = serial.Serial(serial_port, 9600, timeout=1)
time.sleep(2)
# Load the YOLOv8 model
model = YOLO(r"train_1.pt") 
print('communication success')

kinect = Kinect2(use_sensors=['color', 'depth'])
kinect.connect()
kinect.wait_for_worker()

video_path = 1
cap = cv2.VideoCapture(video_path)


def mapping(mid_x,mid_y):
    add_1x = 100
    add_1y = 200
    add_2x = 700
    add_2y = 800
    out_x = add_1x + (mid_x-0)/(600-0)*(add_2x-add_1x)
    out_y = add_1y + (mid_y-0)/(600-0)*(add_2y-add_1y)
    return [out_x,out_y]


def sending_point(list_points):
    if len(list_points)>0:
        sorted_list = sorted(list_points, key=lambda x: x[1])   
        return sorted_list[0]
    else: return None


print("started")
# Loop through the video frames
while cap.isOpened():
    midpoints = []
    main_points = []
    # Read a frame from the video
    success, frame = cap.read()
    frame = cv2.resize(frame,(424,512))

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)
        # Visualize the results on the frame
        frame_plot = results[0].plot()
        # cv2.putText(frame_plot,"Count => "  + str(len(results[0].boxes.cls.tolist())),(50,50),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),2) 
        for i in results[0].boxes.xyxy:
            mid_x = float((i[0]+i[2])/2)
            mid_y = float((i[1]+i[3]/2))
            midpoints.append([mid_x,mid_y])
            # main_points.append(mapping(mid_x,mid_y))
        send_point  = sending_point(midpoints) 
        depth_image = kinect.get_depth_map()
        if send_point != None:
            x,y = int(send_point[0]),int(send_point[1])
            print(x,y)
            print(depth_image[x][y])
        
        if send_point != None:
            data_to_send = str(int(send_point[0])) +','+str(int(send_point[1]))+"," + str(depth_image[x][y]) + "\n"  # Convert the number to a string and add a newline character
            ser.write(data_to_send.encode())
            print(send_point[0],send_point[1],10)
            time.sleep(0.1)
        

        cv2.imshow("YOLOv8 Inference", frame_plot)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            ser.close()
            break
            
    else:
        # Break the loop if the end of the video is reached
        break



# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
ser.close()




