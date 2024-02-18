from djitellopy import Tello
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from ultralytics import YOLO
import os
from cropping import detect_arrow, classify_direction, important_edge, control_cmds
import tkinter as tk

# Create Tello Object
tello = Tello()

# Connect to Tello
tello.connect()
print(f"Battery Life Percentage: {tello.get_battery()}")

# Start the video stream
tello.streamon()
frame_reader = tello.get_frame_read()

# create interrupt
ended = False
def key_press(event):
    global ended
    global tello
    global frame_reader
    if event.char == 'q':
        ended = True
    if event.char == "r":
        tello.streamoff()
        time.sleep(1)
        tello.streamon()
        frame_reader = tello.get_frame_read()


root = tk.Tk()
root.title("Keyboard Interrupts")

canvas = tk.Canvas(root, width=400, height=400)
canvas.pack()
canvas.focus_set()
canvas.bind("<KeyPress>", key_press)

# load models
model_path = os.path.join('.', 'model_weights', 'multi_detect_best.pt')
model_detect = YOLO(model_path) # load trained model

model_path = os.path.join('.', 'model_weights', 'multi_classify_best.pt')
model_classify = YOLO(model_path) # load trained model

tello.takeoff()

# class_id = None
ended = False
def main_loop():
    global ended

    print("Entering While ...")
    no_detections = 0
    while not ended:

        root.update_idletasks()
        root.update()

        img = frame_reader.frame

        box = detect_arrow(model_detect, img)

        if box is not None:

            area = abs((box[2]-box[0]) * (box[3] - box[1]))
            class_id = classify_direction(model_classify, img, box)

            if class_id is not None:

                edge = important_edge(class_id, box)

                forward, dist = control_cmds(area, edge, class_id)
                print(f"Command: dir = {class_id}, dist = {dist}, forward = {forward}")

                # direction is alphabetical ["down", "left", "right", "up"]
                if class_id == 0:  # down
                    if dist != 0:
                        tello.move_down(dist)
                    if forward != 0:
                        tello.move_forward(forward)
                elif class_id == 1:  # left
                    if dist != 0:
                        tello.move_left(dist)
                    if forward != 0:
                        tello.move_forward(forward)
                elif class_id == 2:  # right
                    if dist != 0:
                        tello.move_right(dist)
                    if forward != 0:
                        tello.move_forward(forward)
                else :  # up
                    if dist != 0:
                        tello.move_up(dist)
                    if forward != 0:
                        tello.move_forward(forward)
                
            no_detections = 0

            fig, ax = plt.subplots(figsize=(9,5))
            ax.imshow(img, cmap="gray")
            rect = patches.Rectangle((box[0], box[1]), abs(box[2]-box[0]), abs(box[3]-box[1]), linewidth=1, edgecolor='k', facecolor='none')
            ax.add_patch(rect)
            fig.suptitle(f"Detect Arrow = {class_id}", fontsize= 44)
            plt.show()

        else:
            
            if no_detections > 3 :
                print("No detections 3 frames in a row")
                print("Terminating ...")
                ended = True
    
    print("Terminating ...")
    root.destroy()

root.after(0, main_loop)
root.mainloop()

print("Turning off Tello")
tello.streamoff()
tello.land()


