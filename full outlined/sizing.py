import os
import cv2
import numpy as np
from djitellopy import Tello
import time

# test_path = "./data/images/test"

# count = 0
# for image_file in os.listdir(test_path):
#     image_path = os.path.join(test_path, image_file)

#     image = cv2.imread(image_path)

#     print(image_file)
#     print(image.shape)

#     if count > 10:
#         break

#     count += 1

tello = Tello()

# Connect to Tello
tello.connect()

time.sleep(3)

print(f"Battery Life Percentage: {tello.get_battery()}")


