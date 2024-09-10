from typing import List, Tuple

import numpy as np
import  matplotlib.pyplot as plt

NUM_SAMPLES = 10000


class Ranges:
    def __init__(self):
        self.x_range = [-1, 1]
        self.y_range = [-0.2, 0.22]
        self.theta_range = [0, 6]
        self.gx = -10
        self.gy = 20

def gamma1(gx, gy, theta):
    return np.array([gx * np.cos(theta) - gy * np.sin(theta), gx * np.sin(theta) + gy * np.cos(theta)])

def get_atan(gx, gy):
    atan_gygx = None
    if gx > 0 and gy > 0:
        atan_gygx = np.arctan(gy / gx)
    elif gx > 0 and gy < 0:
        atan_gygx = -np.arctan(-gy / gx)
    elif gx < 0 and gy > 0:
        atan_gygx = -np.arctan(-gy/gx) - np.pi
    elif gx < 0 and gy < 0:
        atan_gygx = np.arctan(gy / gx) + np.pi
    return atan_gygx

def gamma2(gx, gy, theta):
    norm_gxy = np.linalg.norm([gx, gy])
    theta += get_atan(gx, gy)
    return np.array([norm_gxy * np.cos(theta), norm_gxy * np.sin(theta)])

def sample_range(ranges) -> np.array:
    points = np.zeros((NUM_SAMPLES, 3))
    points[:, 0] = np.random.uniform(ranges.x_range[0], ranges.x_range[1], NUM_SAMPLES)
    points[:, 1] = np.random.uniform(ranges.y_range[0], ranges.y_range[1], NUM_SAMPLES)
    points[:, 2] = np.random.uniform(ranges.theta_range[0], ranges.theta_range[1], NUM_SAMPLES)
    return points

def shift_range(ranges: Ranges) -> List[Tuple[float]]:
    atan_gygx = get_atan(ranges.gx, ranges.gy)
    theta1 = ranges.theta_range[0] + atan_gygx
    theta2 = ranges.theta_range[1] + atan_gygx
    thetas = []
    print("shift range", theta1, theta2)

    while theta1 < 0 and theta2 < 0:
        theta1 += 2 * np.pi
        theta2 += 2 * np.pi
    
    while theta1 > 2 * np.pi and theta2 > 2 * np.pi:
        theta1 -= 2 * np.pi
        theta2 -= 2 * np.pi

    if theta1 < 0:
        thetas.append((0, theta2))
        thetas.append((2 * np.pi + theta1, 2 * np.pi))
    elif theta2 > 2 * np.pi:
        thetas.append((0, theta2 - 2 * np.pi))
        thetas.append((theta1, 2 * np.pi))
    else:
        thetas.append((theta1, theta2))
    return thetas

def get_bounding_boxes(ranges: Ranges):
    thetaL = [0, 0.5 * np.pi, np.pi, 1.5 * np.pi]
    thetaR = [0.5 * np.pi, np.pi, 1.5 * np.pi, 2 * np.pi]
    thetas = shift_range(ranges)
    norm_gxy = np.linalg.norm([ranges.gx, ranges.gy])

    fwds = []

    for i in range(4):
        for j in range(len(thetas)):
            print(thetas[j])
            if thetaL[i] > thetas[j][1] or thetas[j][0] > thetaR[i]:
                continue
            theta1_ = max(thetas[j][0], thetaL[i])
            theta2_ = min(thetas[j][1], thetaR[i])

            minX = ranges.x_range[0] + norm_gxy * min(np.cos(theta1_), np.cos(theta2_))
            maxX = ranges.x_range[1] + norm_gxy * max(np.cos(theta1_), np.cos(theta2_))
            minY = ranges.y_range[0] + norm_gxy * min(np.sin(theta1_), np.sin(theta2_))
            maxY = ranges.y_range[1] + norm_gxy * max(np.sin(theta1_), np.sin(theta2_))

            fwds.append((minX, maxX, minY, maxY))
    
    return fwds

def get_global_bounding_box(ranges: Ranges):
    # maxX
    thetas = [ranges.theta_range[0], ranges.theta_range[1]]
    for k in range(-3, 3):
        tmp = np.arctan(-ranges.gy / ranges.gx) + k * np.pi;
        if tmp >= thetas[0] and tmp <= thetas[1]:
            thetas.append(tmp)
    print(thetas)
    minX = ranges.x_range[0] + min(map(lambda t: ranges.gx * np.cos(t) - ranges.gy * np.sin(t), thetas))
    maxX = ranges.x_range[1] + max(map(lambda t: ranges.gx * np.cos(t) - ranges.gy * np.sin(t), thetas))

    # maxY
    thetas = [ranges.theta_range[0], ranges.theta_range[1]]
    for k in range(-3, 3):
        tmp = np.arctan(ranges.gx / ranges.gy) + k * np.pi
        if tmp >= thetas[0] and tmp <= thetas[1]:
            thetas.append(tmp)
    minY = ranges.y_range[0] + min(map(lambda t: ranges.gx * np.sin(t) + ranges.gy * np.cos(t), thetas))
    maxY = ranges.y_range[1] + max(map(lambda t: ranges.gx * np.sin(t) + ranges.gy * np.cos(t), thetas))
    return [(minX, maxX, minY, maxY)]


def draw_box(minX, maxX, minY, maxY):
    # Draw red box
    plt.plot([minX, maxX], [minY, minY], color='red')
    plt.plot([minX, maxX], [maxY, maxY], color='red')
    plt.plot([minX, minX], [minY, maxY], color='red')
    plt.plot([maxX, maxX], [minY, maxY], color='red')



if __name__ == "__main__":
    ranges = Ranges()
    points = sample_range(ranges)
    new_points1 = []
    new_points2 = []
    for idx in range(NUM_SAMPLES):
        new_points1.append(points[idx, :2] + gamma1(ranges.gx, ranges.gy, points[idx, 2]))
        new_points2.append(points[idx, :2] + gamma2(ranges.gx, ranges.gy, points[idx, 2]))
    new_points1 = np.array(new_points1)
    new_points2 = np.array(new_points2)

    X = new_points1[:, 0]
    Y = new_points1[:, 1]
    plt.scatter(X, Y)
    X = new_points2[:, 0]
    Y = new_points2[:, 1]
    # draw much smaller dot size
    plt.scatter(X, Y, s=0.5)

    fwds = get_global_bounding_box(ranges)
    for box in fwds:
        draw_box(box[0], box[1], box[2], box[3])
    print(fwds)

    plt.show()

    

