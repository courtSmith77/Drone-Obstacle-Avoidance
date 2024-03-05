from ultralytics import YOLO
import os
import cv2
from helper_crop import perspective_transform

model_path = "./runs/detect/train/weights/best.pt"
model = YOLO(model_path) # load trained model

test_path = "./data/images/test"

## get all files in the test files:
for image_file in os.listdir(test_path):
    image_path = os.path.join(test_path, image_file)
    results = model.predict(image_path)

    image = cv2.imread(image_path)

    class_id = None
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        
    if class_id is not None:

        yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

        # crop the image
        transformed = perspective_transform(image, yolo_corners)
        flipped = cv2.flip(transformed, -1)
        gray = cv2.cvtColor(flipped, cv2.COLOR_BGR2GRAY)

        new_path = os.path.join(".", "cropped", image_file)
        cv2.imwrite(new_path, gray)
