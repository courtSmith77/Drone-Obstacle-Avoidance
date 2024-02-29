from djitellopy import Tello
import time

tello = Tello()
tello.connect()

print(f"Battery Life Percentage: {tello.get_battery()}")

# tello.takeoff()

# time.sleep(2)

# print("########## moving UP")
# tello.go_xyz_speed(0, 0, 50, 15)

# time.sleep(2)

# print("########## moving DOWN")

# tello.go_xyz_speed(0, 0, -50, 15)

# time.sleep(2)

# print("########## moving RIGHT")

# tello.go_xyz_speed(0, -50, 0, 15)

# time.sleep(2)

# print("########## moving LEFT")

# tello.go_xyz_speed(0, 50, 0, 15)

# time.sleep(2)

# tello.land()