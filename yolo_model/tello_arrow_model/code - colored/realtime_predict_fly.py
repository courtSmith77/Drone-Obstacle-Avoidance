from djitellopy import Tello
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os


model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'best.pt')

model = YOLO(model_path) # load trained model

# Create Tello Object
tello = Tello()

# Connect to Tello
tello.connect()

time.sleep(3)

print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video Stream
tello.streamon()

# Get the frame reader
frame_reader = tello.get_frame_read()

tello.takeoff()
print("############ took off")
time.sleep(2)
tello.move_up(25)
print("############ initial move up")
time.sleep(2)

class_id = None
count = 0
while True:
    # In reality you want to display frames in a separate thread. Otherwise
    # they will freeze while the drone moves.

    print('Before reading image frame')

    # Read a video frame from Tello
    img = frame_reader.frame

    if count > 1:
        print('Running Model')

        results = model.predict(img)

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

        if class_id is not None:
            print("Direction detected :")
            if class_id == 3:
                classify = "Up"
                print("Up")
                tello.move_up(40)
            elif class_id == 0:
                print("Down")
                classify = "Down"
                tello.move_down(30)
            elif class_id == 2:
                classify = "Left"
                tello.move_left(30)
                print("Left")
            else :
                classify = "Right"
                print("Right")
                tello.move_right(40)
            
            # print("Before showing image via Matplotlib")

            # fig, ax = plt.subplots(figsize=(18,10))
            # ax.imshow(img, cmap="gray")
            # rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
            # ax.add_patch(rect)

            # fig.suptitle(classify, fontsize= 44)
            # plt.show()

            time.sleep(4)

            print("Recentering")
            if class_id == 3:
                tello.move_down(30)
            elif class_id == 0:
                tello.move_up(30)
            elif class_id == 2:
                tello.move_right(30)
            else:
                tello.move_left(40)

            # tello.move_up(40)

            # time.sleep(3)

        else :

            print("No Detections")
            plt.imshow(img, cmap="gray")
            plt.show()

        print("########### Change Direction NOW !!")
        time.sleep(15)

    if count > 3:
        break

    count+=1

# cv2.destroyWindow('Tello View')
# cv2.destroyAllWindows()
tello.streamoff()

tello.land()


    


