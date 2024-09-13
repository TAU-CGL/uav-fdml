import time
import threading

import djitellopy

def tof_thread():
    while True:
        try:
            print("TOF::\t",time.time(), drone.get_distance_tof())
            time.sleep(1.0 / 60.0)
        except:
            break


# # Tello object
drone = djitellopy.Tello()

drone.connect()

time.sleep(5)
drone.set_speed(10)
drone.takeoff()

# Start TOF thread
tof_thread = threading.Thread(target=tof_thread)
tof_thread.start()


time.sleep(2)

drone.move_up(70)

print(f"[FWD 400 @ {time.time()}]")
drone.move_forward(400)
print(f"--[FWD 400 @ {time.time()}]")

time.sleep(2)

print(f"[RIGHT 120 @ {time.time()}]")
drone.move_right(120)
print(f"--[RIGHT 120 @ {time.time()}]")

time.sleep(2)

print(f"[BCK 400 @ {time.time()}]")
drone.move_back(400)
print(f"--[BCK 400 @ {time.time()}]")

drone.land()
drone.disconnect()
tof_thread.join()