from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch
# ^^ using the yolov8 nano model

# Use the model
model.train(data="trainyaml.yaml", epochs=500)  # train the model

# metrics = model.val()
# print(metrics)