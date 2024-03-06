from djitellopy import Tello
import cv2
from ultralytics import YOLO
import os
from model_dev.helper_crop import perspective_transform

model_path = os.path.join('..', 'model_dev', 'runs', 'detect', 'train', 'weights', 'best.pt')
model_detect = YOLO(model_path) # load trained model

model_path = os.path.join('..', 'model_dev',  'runs', 'classify', 'train', 'weights', 'best.pt')
model_classify = YOLO(model_path) # load trained model

# Create Tello Object
tello = Tello()

# Connect to Tello
tello.connect()

print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video Stream
tello.streamon()

class_id = None
while True:

    frame_reader = tello.get_frame_read()
    img = frame_reader.frame

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

            print(f'Arrow Direction: {classify_id}')



