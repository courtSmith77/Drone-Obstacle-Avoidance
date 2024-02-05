from djitellopy import Tello
import time

tello = Tello()
tello.connect()

print(f"Battery Life Percentage: {tello.get_battery()}")

# tello.takeoff()

# time.sleep(2)

# tello.move_up(30)

# print("########## moving UP")

# time.sleep(2)

# tello.move_down(30)

# print("########## moving DOWN")

# time.sleep(2)

# tello.move_right(30)

# print("########## moving RIGHT")

# time.sleep(2)

# tello.move_left(30)

# print("########## moving LEFT")

# time.sleep(2)

# tello.land()