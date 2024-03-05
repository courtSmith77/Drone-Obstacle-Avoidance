from djitellopy import Tello
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os


model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'best.pt')

model = YOLO(model_path) # load trained model

tello = Tello()

tello.connect()

print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video Stream
tello.streamon()
frame_reader = tello.get_frame_read()

class_id = None
count = 0
while True:

    # Read a video frame from Tello
    img = frame_reader.frame

    if count > 1:
        print('Running Model')

        results = model.predict(img)

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

        if class_id is not None:

            fig, ax = plt.subplots(figsize=(18,10))
            ax.imshow(img, cmap="gray")
            rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=3, edgecolor='k', facecolor='none')
            ax.add_patch(rect)
            plt.show()
           
        else :

            print("No Detections")
            plt.imshow(img, cmap="gray")
            plt.show()

    if count > 5:
        break

    count+=1

tello.streamoff()


    


