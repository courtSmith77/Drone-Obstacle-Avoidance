# Testing YOLO Models

- `realtime_predict_no_fly` - Tests the detection algorithm in realtime when connected to the Tello drone. Plots the detection box on the live drone image.
- `yolo_cascade.py` - Tests the full detection and classification cascade on test images. Plots the detection box and cropped classified image.
- `realtime_cascade_no_fly` - Tests the full detection and classification cascade in realtime when connected to the Tello drone. Prints the detected arrow direction to the terminal. Only checks one classifier at a time.