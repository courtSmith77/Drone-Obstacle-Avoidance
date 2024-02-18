from djitellopy import Tello
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os
from cropping import perspective_transform


model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'best.pt')
model_detect = YOLO(model_path) # load trained model

model_path = os.path.join('.', 'runs', 'classify', 'train', 'weights', 'best.pt')
model_classify = YOLO(model_path) # load trained model

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

        results = model_detect.predict(img)

        detected = None
        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, detected = result

        if detected is not None:

            yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

            transformed = perspective_transform(img, yolo_corners)
            flipped = cv2.flip(transformed, -1)

            print(f"Flipped depth dtype = {flipped.dtype}")

            pred = model_classify.predict(flipped)

            class_id = None
            if pred is not None:
                
                class_id = pred[0].probs.top1
                classify_id = pred[0].names[class_id]

                if class_id is not None:
                    print(class_id)

                    fig, ax = plt.subplots(figsize=(9,5))
                    ax.imshow(img, cmap="gray")
                    rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
                    ax.add_patch(rect)
                    fig.suptitle(f"Detect Arrow", fontsize= 44)

                    fig1,ax1 = plt.subplots(figsize=(9,5))
                    ax1.imshow(flipped)
                    fig1.suptitle(f"Classify Arrow - {classify_id}", fontsize= 44)
                    plt.show()
            else:
                fig, ax = plt.subplots(figsize=(9,5))
                ax.imshow(img, cmap="gray")
                rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
                ax.add_patch(rect)
                fig.suptitle(f"Detect Arrow", fontsize= 44)

                fig1,ax1 = plt.subplots(figsize=(9,5))
                ax1.imshow(flipped)
                fig1.suptitle(f"No Class Detected", fontsize= 44)
                plt.show()


    if count > 7:
        break

    count+=1

tello.streamoff()