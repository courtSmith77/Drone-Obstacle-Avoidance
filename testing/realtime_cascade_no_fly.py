from djitellopy import Tello
import cv2
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

print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video Stream
tello.streamon()


class_id = None
count = 0
while True:

    frame_reader = tello.get_frame_read()
    img = frame_reader.frame


    if count > 1:

        results = model_detect.predict(img)

        detected = None
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

    if count > 5:
        break

    count+=1
                    

tello.streamoff()


