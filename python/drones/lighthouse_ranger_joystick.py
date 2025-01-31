import os
import json
import time
import math
import threading

import pygame
import requests

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Initialize Pygame
pygame.init()

# Set up the joystick
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

SERVER_URL = f"{os.environ['FDML_SERVER']}:9989" # Notice that no need for port in the enrionment variable
URI = 'radio://0/76/2M/E7E7E7E703' # Change this to your Crazyflie's URI
DEFAULT_HEIGHT = 0.5
SENSITIVITY = 0.5
INTERVAL = 0.1
THRESHOLD = 0.2

DEFAULT_FOLDER = "cfresults"

all_data = []

def send_request(data):
    data["type"] = "crazyflie"
    requests.post(f"{SERVER_URL}/post_measurement", json={"measurement": data})

def log_callback(timestamp, data, logconf):
    global all_data

    front = data["range.front"]
    back = data["range.back"]
    right = data["range.right"]
    left = data["range.left"]
    x = data["stateEstimate.x"]
    y = data["stateEstimate.y"]
    z = data["stateEstimate.z"]
    yaw = data["stateEstimate.yaw"]
    data = {"front": front / 1000 if front != 32766 else -1, 
            "back": back / 1000 if back != 32766 else -1,
            "right": right / 1000 if right != 32766 else -1,
            "left": left / 1000 if left != 32766 else -1,
            "x": x, "y": y, "z": z, "yaw": yaw / 180 * math.pi + math.pi}
    all_data.append(data)
    # print(x, y, z)
    thread = threading.Thread(target=send_request, args=(data,))
    thread.start()

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_callback)
    logconf.start()
    return logconf

def control(scf, lg):
    print("starting control")
    cf = scf.cf
    commander = cf.high_level_commander

    commander.takeoff(DEFAULT_HEIGHT, 3)
    time.sleep(3.2)

    l = simple_log_async(scf, lg)
    
    run = True

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(1):
                    run = False
                elif joystick.get_button(2):
                    commander.go_to(0, 0, DEFAULT_HEIGHT, 0.0, 3.0)
                    time.sleep(3.1)
        y = -joystick.get_axis(0)
        if abs(y) < THRESHOLD:
            y = 0
        x = -joystick.get_axis(1)
        if abs(x) < THRESHOLD:
            x = 0
        yaw = joystick.get_axis(2)
        if abs(yaw) < THRESHOLD:
            yaw = 0
        yaw *=  math.pi / 2
        z = -joystick.get_axis(3)
        if abs(z) < THRESHOLD:
            z = 0
        commander.go_to(x * SENSITIVITY * INTERVAL, y * SENSITIVITY * INTERVAL, z * SENSITIVITY * INTERVAL, yaw * SENSITIVITY * INTERVAL, INTERVAL, relative=True)
        # time.sleep(INTERVAL)
        pygame.time.wait(int(INTERVAL * 1000))

    l.stop()
    pygame.quit()
    commander.land(0.0, 3.0)
    time.sleep(3.2)
    commander.stop()


if __name__ == '__main__':
    folder = input(f"Enter folder (default: {DEFAULT_FOLDER}): ")
    if folder == "":
        folder = DEFAULT_FOLDER

    cflib.crtp.init_drivers()

    lg = LogConfig(name='Logger', period_in_ms=10)
    lg.add_variable('range.back', 'uint16_t')
    lg.add_variable('range.front', 'uint16_t')
    lg.add_variable('range.left', 'uint16_t')
    lg.add_variable('range.right', 'uint16_t')
    lg.add_variable('range.up', 'uint16_t')

    lg.add_variable('stateEstimate.x', 'float')
    lg.add_variable('stateEstimate.y', 'float')
    lg.add_variable('stateEstimate.z', 'float')
    lg.add_variable('stateEstimate.yaw', 'float')

    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(URI) as scf:
        scf.cf.param.set_value('commander.enHighLevel', '1')
        control(scf, lg)
        if not os.path.isfile(os.path.join(folder, "index.txt")):
            with open(os.path.join(folder, "index.txt"), 'w') as f:
                f.write("1")
            print("Index file didn't exist, created new one")
        with open(os.path.join(folder, "index.txt"), 'r') as f:
            id = f.read().strip()
        with open(os.path.join(folder, f"measurements_{id}.json"), 'w') as f:
            json.dump(all_data, f)
        with open(os.path.join(folder, "index.txt"), 'w') as f:
            f.write(str(int(id) + 1))
        print(f"Done! Got {len(all_data)} measurements")
