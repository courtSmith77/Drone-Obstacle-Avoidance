from ultralytics import YOLO
import torch

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch
# ^^ using the yolov8 nano model

if torch.cuda.is_available():

    # Use the model with cuda 
    model.train(data="trainyaml.yaml", epochs=500, device=0)  # train the model

else :
    model.train(data="trainyaml.yaml", epochs=500)  # train the model

# metrics = model.val()
# print(metrics)
