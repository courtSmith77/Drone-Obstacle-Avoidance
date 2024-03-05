# Model Training

## Dection Model
- Detection data:
    - Annotate each image with a bounding box indicating a symbol (recommend using CVAT)
    - Place files in specified structure
    - Update `detection_train_params.yaml` to have the absolute path to your data_detection file

    ├─ data_detection
       ├─ images
       |   ├─ test
       |   |  └─ images.jpg
       |   └─ train
       |      └─ images.jpg
       └─ labels
           ├─ test
           |  └─ images.jpg
           └─ train
              └─ images.jpg

- Training model:
    - Run `yolo_train_detection.py` to train the model
    - Model analysis metrics and final weights will be found in the newly creates `run` folder

## Classification Model
- Classification data:
    - Crop each image using `crop_images_with_detector.py` the trained detection model and place in following structure

    ├─ data_classification
       ├─ test
       |   ├─ class_label1
       |   |  └─ images.jpg
       |   ├─ class_label2
       |   |  └─ images.jpg
       |   └─ class_label3
       |      └─ images.jpg
       └─ train
           ├─ class_label1
           |  └─ images.jpg
           ├─ class_label2
           |  └─ images.jpg
           └─ class_label3
              └─ images.jpg



- yaml file (absolute path)
- creating images for the classifier from the detection

## YOLO
- detection
    - uses yaml
- classify
    - uses model structure
- results

