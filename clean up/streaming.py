import cv2
from djitellopy import tello

drone = tello.Tello()
drone.connect()
drone.streamon()

while True:

    print('In while')
    frame = drone.get_frame_read().frame

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()