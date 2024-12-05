import math

import vpython as vp

PRONG_LEN = 76.87 + 10.17
K = 16
D = 100

def generate_odometries():
    gs = []
    for i in range(K):
        t = i * 2 * math.pi / K
        x = PRONG_LEN * math.cos(t)
        y = PRONG_LEN * math.sin(t)
        z = 0
        vx =  2 ** 0.5 * 0.5; vy = 0; vz = -2 ** 0.5 * 0.5
        vx_ = vx * math.cos(t) - vy * math.sin(t)
        vy_ = vx * math.sin(t) + vy * math.cos(t)
        vz_ = vz
        gs.append((x,y,z,vx_,vy_,vz_))

    return gs


def FdgV(q, g, d):
    x, y, z, t = q
    gx, gy, gz, vx, vy, vz = g

    x_ = x + math.cos(t) * (gx + d * vx) - math.sin(t) * (gy + d * vy)
    y_ = y + math.sin(t) * (gx + d * vx) + math.cos(t) * (gy + d * vy)
    z_ = z + gz + d * vz

    return (x_, y_, z_)

if __name__ == "__main__":
    scene = vp.canvas(title="FdgV", width=800, height=600, background=vp.color.white)

    # Generate (fixed) g_is
    gs = generate_odometries()
    theta = 0
    
    # Visualization
    ball = vp.sphere(pos=vp.vector(0, 0, 0), radius=5, color=vp.color.red)
    sensor_arrows = []
    ray_arrows = []
    for g in gs:
        x, y, z, vx, vy, vz = g
        x_ = math.cos(theta) * x - math.sin(theta) * y
        y_ = math.sin(theta) * x + math.cos(theta) * y
        sensor_arrows.append(vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(x_, y_, z), color=vp.color.red, shaftwidth=1))

        x__, y__, z__ = FdgV((0, 0, 0, theta), g, D)
        ray_arrows.append(vp.arrow(pos=vp.vector(x_, y_, z), axis=vp.vector(x__ - x_, y__ - y_, z__ - z), color=vp.color.green, shaftwidth=1))


    dt = 0.01
    while True:
        vp.rate(100)

        theta += 50 / 180 * math.pi * dt

        for i, g in enumerate(gs):
            x, y, z, vx, vy, vz = g
            x_ = math.cos(theta) * x - math.sin(theta) * y
            y_ = math.sin(theta) * x + math.cos(theta) * y
            sensor_arrows[i].axis = vp.vector(x_, y_, z)

            x__, y__, z__ = FdgV((0, 0, 0, theta), g, D)
            ray_arrows[i].pos = vp.vector(x_, y_, z)
            ray_arrows[i].axis = vp.vector(x__ - x_, y__ - y_, z__ - z)
            # ray_arrows[i].pos = vp.vector(0,0,0)
            # ray_arrows[i].axis = vp.vector(x__, y__, z__)
