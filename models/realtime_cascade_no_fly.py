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

# H, W, _ = tello.get_frame_read().frame.shape
# output = cv2.VideoWriter()
# four = cv2.VideoWriter_fourcc(*'DIVX')
# output = cv2.VideoWriter('streaming_04.avi', four, 30, (W, H), True)

class_id = None
count = 0
while True:
    # In reality you want to display frames in a separate thread. Otherwise
    # they will freeze while the drone moves.

    # print('Before reading image frame')

    # Read a video frame from Tello
    # Get the frame reader
    frame_reader = tello.get_frame_read()
    img = frame_reader.frame

    # output.write(img)

    if count > 1:
        # print('Running Model')

        results = model_detect.predict(img)

        detected = None
        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, detected = result

        if detected is not None:

            yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

            transformed = perspective_transform(img, yolo_corners)
            flipped = cv2.flip(transformed, -1)

            # img = cv2.rectangle(img, (x1,y2), (x2,y1), (0,0,0), 2)

            pred = model_classify.predict(flipped)

            class_id = None
            if pred is not None:
                
                class_id = pred[0].probs.top1
                classify_id = pred[0].names[class_id]

    if count > 5:
        break

    count+=1
                    

# output.release()
tello.streamoff()


