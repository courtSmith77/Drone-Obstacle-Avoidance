from ultralytics import YOLO
import torch
import os
import cv2
from cropping import perspective_transform
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from trainCNN_pt import CNNModel

# Set device (GPU if available, else CPU)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# YOLO Model
model_path = "./runs/detect/train/weights/best.pt"
model_yolo = YOLO(model_path) # load trained model

# Image folder
test_path = "./data/images/train"

# CNN Model
model_path = "./cnn_model.pth"
model_cnn = CNNModel().to(device)
model_cnn.load_state_dict(torch.load(model_path))

IMG_SIZE = 200

count = 0
## get all files in the test files:
for image_file in os.listdir(test_path):
    image_path = os.path.join(test_path, image_file)
    results = model_yolo.predict(image_path)

    image = cv2.imread(image_path)

    class_id = None
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        
    if class_id is not None:

        yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

        transformed = perspective_transform(image, yolo_corners)
        flipped = cv2.flip(transformed, -1)

        resized = cv2.resize(flipped, (IMG_SIZE, IMG_SIZE))
        resized = np.array(resized).reshape(-1, IMG_SIZE, IMG_SIZE, 3)
        img = resized.astype('float32') / 255.0
        img_tensor = torch.tensor(img.transpose((0,3,1,2)), dtype=torch.float32, device=device)

        pred = model_cnn.forward(img_tensor)
        _, dir = torch.max(pred, axis=1)

        # if class_id == 3:
        #     classify_yolo = "Up"
        #     print("Up")
        # elif class_id == 0:
        #     classify_yolo = "Down"
        #     print("Down")
        # elif class_id == 2:
        #     classify_yolo = "Left"
        #     print("Left")
        # else :
        #     classify_yolo = "Right"
        #     print("Right")

        if dir == 0:
            classify_cnn = "Up"
            print("Up")
        elif dir == 1:
            classify_cnn = "Down"
            print("Down")
        elif dir == 2:
            classify_cnn = "Left"
            print("Left")
        else :
            classify_cnn = "Right"
            print("Right")

        fig, ax = plt.subplots(figsize=(9,5))
        ax.imshow(image, cmap="gray")
        rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
        ax.add_patch(rect)
        fig.suptitle(f"YOLO Arrow", fontsize= 44)

        fig1,ax1 = plt.subplots(figsize=(9,5))
        ax1.imshow(flipped)
        fig1.suptitle(f"CNN Arrow - {classify_cnn}", fontsize= 44)
        plt.show()


    if count > 20:
        break
    print(count)
    count += 1