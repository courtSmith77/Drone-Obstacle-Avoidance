from ultralytics import YOLO
import torch

# Load pretrained yolov8 nano model
model = YOLO("yolov8n-cls.pt")

if torch.cuda.is_available():
    model.train(data="./data_class_arrows", epochs=500, device=0)
else :
    model.train(data="./data_class_arrows", epochs=500)
