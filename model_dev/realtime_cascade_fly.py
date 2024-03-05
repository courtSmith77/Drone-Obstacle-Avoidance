from djitellopy import Tello
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os
from cropping import perspective_transform


model_path = os.path.join('.', 'runs', 'detect', 'train', 'weights', 'best.pt')
model_detect = YOLO(model_path) # load trained model

model_path = os.path.join('.', 'runs', 'classify', 'train', 'weights', 'best.pt')
model_classify = YOLO(model_path) # load trained model

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
time.sleep(4)
# tello.move_up(25)
# print("############ initial move up")
# time.sleep(2)

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

        results = model_detect.predict(img)

        for result in results[0].boxes.data.tolist():
            x1, y1, x2, y2, score, detected = result
        
        if detected is not None:

            yolo_corners = [[x1,y2],[x2,y2],[x2,y1],[x1,y1]] # top_l, top_r, bot_r, bot_l

            transformed = perspective_transform(img, yolo_corners)
            flipped = cv2.flip(transformed, -1)

            pred = model_classify.predict(flipped)

            class_id = None
            if pred is not None:
                
                class_id = pred[0].probs.top1
                classify_id = pred[0].names[class_id]

                if class_id is not None:
                    print("Direction detected :")
                    if class_id == 3:
                        print("Up")
                        print("Moving Up")
                        tello.move_up(40)
                        print("Up Complete")
                        print("Moving Forward")
                        tello.move_forward(20)
                        print("Forward Complete")
                    elif class_id == 0:
                        print("Down")
                        print("Moving Down")
                        tello.move_down(30)
                        print("Down Complete")
                        print("Moving Forward")
                        tello.move_forward(20)
                        print("Forward Complete")
                        # break
                    elif class_id == 1:
                        print("Left")
                        print("Moving Left")
                        tello.move_left(30)
                        print("Left Complete")
                        print("Moving Forward")
                        tello.move_forward(20)
                        print("Forward Complete")
                        # break
                    else :
                        print("Right")
                        print("Moving Right")
                        tello.move_right(40)
                        print("Right Complete")
                        print("Moving Forward")
                        tello.move_forward(20)
                        print("Forward Complete")
                        # break
            
                    # print("Before showing image via Matplotlib")

                    time.sleep(4)

                    print("Recentering")
                    if class_id == 3:
                        tello.move_down(30)
                        tello.move_back(20)
                    elif class_id == 0:
                        tello.move_up(30)
                        tello.move_back(20)
                    elif class_id == 1:
                        tello.move_right(30)
                        tello.move_back(20)
                    else:
                        tello.move_left(40)
                        tello.move_back(20)

                    fig, ax = plt.subplots(figsize=(9,5))
                    ax.imshow(img, cmap="gray")
                    rect = patches.Rectangle((x1, y1), abs(x2-x1), abs(y2-y1), linewidth=1, edgecolor='k', facecolor='none')
                    ax.add_patch(rect)
                    fig.suptitle(f"Detect Arrow", fontsize= 44)

                    fig1,ax1 = plt.subplots(figsize=(9,5))
                    ax1.imshow(flipped)
                    fig1.suptitle(f"Classify Arrow - {classify_id}", fontsize= 44)
                    plt.show()

            # tello.move_up(40)

            # time.sleep(3)

        else :

            print("No Detections")
            plt.imshow(img, cmap="gray")
            plt.show()

        # print("########### Change Direction NOW !!")
        # time.sleep(15)

    if count > 3:
        break

    count+=1

# cv2.destroyWindow('Tello View')
# cv2.destroyAllWindows()
tello.streamoff()

tello.land()


    


