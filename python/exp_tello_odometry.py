import time

from djitellopy import Tello

tello = Tello()
tello.connect()
tello.set_speed(10)
tello.takeoff()

time.sleep(5)
tello.move_forward(50)
time.sleep(5)
tello.move_back(50)
time.sleep(5)

tello.land()
tello.end()