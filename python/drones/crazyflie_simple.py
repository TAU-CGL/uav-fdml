# https://gist.github.com/knmcguire/39274011c6dc400ad9534f8a3c078b70


import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander

# Change uris and sequences according to your setup
URI = 'radio://0/76/2M/E7E7E7E701'
# URI = 'radio://0/76/2M/E7E7E7E702'

DEFAULT_HEIGHT = 0.3

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # with MotionCommander(scf) as motion_commander:
        with Multiranger(scf) as multi_ranger:
            # time.sleep(1.0)
            # motion_commander.up(DEFAULT_HEIGHT)
            # time.sleep(2.0)

            while True:
                vals = [
                    multi_ranger.front, multi_ranger.back, multi_ranger.right, multi_ranger.left, multi_ranger.up,
                    multi_ranger.down
                ]
                vals = [str(v) for v in vals]
                print(','.join(vals) + ',')

                time.sleep(0.1)

            # motion_commander.land()
            # motion_commander.stop()
                