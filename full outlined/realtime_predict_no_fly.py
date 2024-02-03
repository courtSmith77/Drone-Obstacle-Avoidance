from djitellopy import Tello
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os


model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'best.pt')

model = YOLO(model_path) # load trained model

# Create Tello Object
tello = Tello()

# Connect to Tello
tello.connect()

print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video Stream
tello.streamon()

# Get the frame reader
frame_reader = tello.get_frame_read()

class_id = None
count = 0
while True:
    # In reality you want to display frames in a separate thread. Otherwise
    # they will freeze while the drone moves.

    print('Before reading image frame')

    # Read a video frame from Tello
    img = frame_reader.frame

    if count > 1:
        print('Running Model')

        results = model.predict(img)

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

        if class_id is not None:
            # print("Direction detected :")
            # if class_id == 3:
            #     classify = "Up"
            #     print("Up")
            # elif class_id == 0:
            #     print("Down")
            #     classify = "Down"
            # elif class_id == 2:
            #     classify = "Left"
            #     print("Left")
            # else :
            #     classify = "Right"
            #     print("Right")

            fig, ax = plt.subplots(figsize=(18,10))
            ax.imshow(img, cmap="gray")
            rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=3, edgecolor='k', facecolor='none')
            ax.add_patch(rect)

            # fig.suptitle(classify, fontsize= 44)
            plt.show()
           
        else :

            print("No Detections")
            plt.imshow(img, cmap="gray")
            plt.show()

    if count > 7:
        break

    count+=1

tello.streamoff()


    


