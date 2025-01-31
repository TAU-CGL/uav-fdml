import time
import random
import logging
from math import pi
import json

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Original Trajectory in Lab 363
# TRAJECTORY = [(0, -1), (1.6, -1), (1.73, -0.8), (0.13, -0.8), (0, -0.6), (1.75, -0.6), (1.78, -0.4), (0, -0.4), (0, -0.2), (1.75, -0.2), (1.61, 0), (-0.1, 0), (0, 0.2), (1.57, 0.2), (1.57, 0.4), (0.05, 0.4), (0, 0.6), (1.55, 0.6), (1.3, 0.8), (0, 0.8)]
# ZMIN = 0.65
# ZMAX = 0.8

# Trajectory in Lab 446 (21/01/2025)
TRAJECTORY = [(0, 0), (-0.04, -0.22), (1.14, -0.28), (1.16, 0.58), (-0.75, 0.57), (-0.32, -0.05), (0, 0)]
ZMIN = 1.25
ZMAX = 1.35

URI = 'radio://0/76/2M/E7E7E7E701'

logging.basicConfig(level=logging.ERROR)

SPEED = 0.2

all_data = []

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
    print(f"front: {front}, back: {back}, right: {right}, left: {left}, x: {x}, y: {y}, z: {z}, yaw: {yaw}")
    if 32766 in [front, back, right, left]:
        return
    all_data.append({"front": front / 1000, "back": back / 1000, "right": right / 1000, "left": left / 1000, "x": x, "y": y, "z": z, "yaw": yaw / 180 * pi + pi})

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_callback)
    logconf.start()
    return logconf

def get_time(start, end):
    return sum((x - y) ** 2 for x, y in zip(start, end)) ** 0.5 / SPEED

def control(scf, lg):
    cf = scf.cf
    commander = cf.high_level_commander

    height = random.uniform(ZMIN, ZMAX)
    commander.takeoff(height, 2.5)
    time.sleep(3)

    commander.go_to(*(TRAJECTORY[0]), height, random.uniform(-pi, pi), 5)
    time.sleep(5.7)

    l = simple_log_async(scf, lg)

    for i, coord in enumerate(TRAJECTORY[1:]):
        t = get_time(TRAJECTORY[i], coord)
        commander.go_to(coord[0] - TRAJECTORY[i][0], coord[1] - TRAJECTORY[i][1], 0, random.uniform(-pi, pi) * (min(t / 8, 1)), t, relative=True)
        time.sleep(t + 0.2)

    l.stop()

    commander.land(0.0, 3)
    time.sleep(3.5)
    commander.stop()


if __name__ == '__main__':
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
        with open('measurements.json', 'w') as f:
            json.dump(all_data, f)