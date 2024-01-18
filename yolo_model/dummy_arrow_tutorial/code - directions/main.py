from ultralytics import YOLO
import torch

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch
# ^^ using the yolov8 nano model

if torch.cuda.is_available():
    print('################ Trying to use cuda ##########')
    # device_name = torch.cuda.get_device_name(0)

    print('################ found gpu ##########')
    # Use the model with cuda 
    model.train(data="trainyaml.yaml", epochs=150, device=0)  # train the model

    print('################ input it here ##########')
else :
    print('################ NOT using cuda ##########')
    model.train(data="trainyaml.yaml", epochs=150)  # train the model

# metrics = model.val()
# print(metrics)