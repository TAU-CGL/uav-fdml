import time

from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()

# tello.move_left(30)
# tello.rotate_counter_clockwise(70)
tello.move_forward(20)
time.sleep(2)
tello.move_back(20)
# tello.rotate_clockwise(70)
# tello.move_right(30)

tello.land()
