from ultralytics import YOLO
import torch
import os
import cv2
from model_dev.helper_crop import perspective_transform
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Set device (GPU if available, else CPU)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# YOLO Detection
model_path = "../model_dev/runs/detect/train/weights/best.pt"
model_yolo = YOLO(model_path)

# Image folder
test_path = "../model_dev/data_detection/images/test"

# YOLO Classifier
class_path = "../model_dev/runs/classify/train/weights/best.pt"
model_class = YOLO(class_path)

count = 0
## get all files in the test files:
for image_file in os.listdir(test_path):
    image_path = os.path.join(test_path, image_file)
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

                fig, ax = plt.subplots(figsize=(9,5))
                ax.imshow(image, cmap="gray")
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


    if count > 10:
        break
    print(count)
    count += 1