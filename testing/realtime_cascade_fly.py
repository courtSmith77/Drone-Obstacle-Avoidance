from djitellopy import Tello
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os
from model_dev.helper_crop import perspective_transform


model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'best.pt')
model_detect = YOLO(model_path) # load trained model

model_path = os.path.join('.', 'runs', 'classify', 'train', 'weights', 'best.pt')
model_classify = YOLO(model_path) # load trained model

# Create Tello Object
tello = Tello()

# Connect to Tello
tello.connect()

time.sleep(1)

print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video Stream
tello.streamon()

# Get the frame reader
frame_reader = tello.get_frame_read()

tello.takeoff()
time.sleep(4)


class_id = None
count = 0
while True:

    # Read a video frame from Tello
    img = frame_reader.frame

    if count > 1:

        results = model_detect.predict(img)

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, detected = result
        
        if detected is not None:

            yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

            transformed = perspective_transform(img, yolo_corners)
            flipped = cv2.flip(transformed, -1)

            pred = model_classify.predict(flipped)

            class_id = None
            if pred is not None:
                
                class_id = pred[0].probs.top1
                classify_id = pred[0].names[class_id]

                if class_id is not None:
                    if class_id == 3:
                        print("Moving Up")
                        tello.move_up(40)
                        print("Up Complete")
                    elif class_id == 0:
                        print("Moving Down")
                        tello.move_down(30)
                        print("Down Complete")
                    elif class_id == 1:
                        print("Moving Left")
                        tello.move_left(30)
                        print("Left Complete")
                    else :
                        print("Moving Right")
                        tello.move_right(40)
                        print("Right Complete")

                    print("Recentering")
                    if class_id == 3:
                        tello.move_down(30)
                        tello.move_back(20)
                    elif class_id == 0:
                        tello.move_up(30)
                        tello.move_back(20)
                    elif class_id == 1:
                        tello.move_right(30)
                        tello.move_back(20)
                    else:
                        tello.move_left(40)
                        tello.move_back(20)

                    fig, ax = plt.subplots(figsize=(9,5))
                    ax.imshow(img, cmap="gray")
                    rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
                    ax.add_patch(rect)
                    fig.suptitle(f"Detect Arrow", fontsize= 44)

                    fig1,ax1 = plt.subplots(figsize=(9,5))
                    ax1.imshow(flipped)
                    fig1.suptitle(f"Classify Arrow - {classify_id}", fontsize= 44)
                    plt.show()

        else :

            print("No Detections")
            plt.imshow(img, cmap="gray")
            plt.show()


    if count > 3:
        break

    count+=1

tello.streamoff()
tello.land()


    


