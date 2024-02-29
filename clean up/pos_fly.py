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

tello.takeoff()

# class_id = None
ended = False
def main_loop():
    global ended

    z = 5
    print("Entering While ...")
    while not ended:

        root.update_idletasks()
        root.update()

        z *= -1
        tello.go_xyz_speed(20, 0, 20, 15)

    print("Terminating ...")
    root.destroy()

root.after(0, main_loop)
root.mainloop()

print("Turning off Tello")
tello.streamoff()
tello.land()


