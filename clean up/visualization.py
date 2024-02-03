from ultralytics import YOLO
from djitellopy import Tello
import torch
import os
import cv2
from cropping import perspective_transform
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class flyTello():

    def __init__(self):

        # initialize drone object
        self.drone = Tello()

        # initialize model
        self.model_detect = YOLO("./model_weights/detect_best.pt")
        self.model_classify = YOLO("./model_weights/classify_best.pt")

        # initialize visualization steps
        self.detect = None
        self.classify = None

    def connect_drone(self):

        self.drone.connect()

        print(f"Battery Life Percentage: {self.drone.get_battery()}")

        # Start the video Stream
        self.drone.streamon()

        # Get the frame reader
        self.frame_reader = self.drone.get_frame_read()


def detect_arrow(model, img):

    results = model.predict(img)

    detected = None
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, detected = result

    if detected == None:
        print("no arrow detected")
        return detected
    else :
        return [x1, y1, x2, y2]
    
    
def classify_direction(model, img, box):

    yolo_corners = [[box[0],box[3]],[box[2],box[3]],[box[2],box[1]],[box[0],box[1]]] # top_l, top_r, bot_r, bot_l

    transformed = perspective_transform(img, yolo_corners)
    flipped = cv2.flip(transformed, -1)

    pred = model.predict(flipped)

    class_id = None
    if pred is not None:
        class_id = pred[0].probs.top1

    return class_id

def object_edge(box, direction):

    # direction is alphabetical ["down", "left", "right", "up"]
    if direction == 0:
        # down = y1
        edge = box[1]
    elif direction == 1:
        # left = x1
        edge = box[0]
    elif direction == 2:
        # right = x2
        edge = box[2]
    else :
        # up = y2
        edge = box[3]

    return edge

def object_depth(box):

    # area = abs((x2-x1)*(y2-y1))
    area = abs((box[2]-box[0])*(box[3]-box[1]))

    depth = area/100

    return depth
