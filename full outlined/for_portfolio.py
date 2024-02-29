from ultralytics import YOLO
import torch
import os
import cv2
from cropping import perspective_transform
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.image as mpimg

# Set device (GPU if available, else CPU)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# YOLO Model
model_path = "./runs/detect/train/weights/best.pt"
model_yolo = YOLO(model_path) # load trained model

# YOLO Classifier
class_path = "./runs/classify/train/weights/best.pt"
model_class = YOLO(class_path)

image_path = "/home/csmith/Desktop/Job Planning/Portfolio/Winter Project/cascade figure/original.jpg"

results = model_yolo.predict(image_path)

image = cv2.imread(image_path)

detected = None
for result in results[0].boxes.data.tolist():
    x1, y1, x2, y2, score, detected = result
    
if detected is not None:

    yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

    transformed = perspective_transform(image, yolo_corners)
    flipped = cv2.flip(transformed, -1)

    pred = model_class.predict(flipped)

    class_id = None
    if pred is not None:
        
        class_id = pred[0].probs.top1
        classify_id = pred[0].names[class_id]

        if class_id is not None:
            
            img = mpimg.imread(image_path)
            fig, ax = plt.subplots(figsize=(9,5))
            ax.imshow(img)
            rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
            ax.add_patch(rect)
            fig.suptitle(f"Detect Arrow", fontsize= 44)

            fig1,ax1 = plt.subplots(figsize=(9,5))
            ax1.imshow(flipped)
            fig1.suptitle(f"Classify Arrow - {classify_id}", fontsize= 44)
            plt.show()
    else:
        fig, ax = plt.subplots(figsize=(9,5))
        ax.imshow(image, cmap="gray")
        rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
        ax.add_patch(rect)
        fig.suptitle(f"Detect Arrow", fontsize= 44)

        fig1,ax1 = plt.subplots(figsize=(9,5))
        ax1.imshow(flipped)
        fig1.suptitle(f"No Class Detected", fontsize= 44)
        plt.show()