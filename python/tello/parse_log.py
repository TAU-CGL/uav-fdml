import matplotlib.pyplot as plt

if __name__ == "__main__":
    log_file = "python/tello/log.txt"
    heights = []
    times = []
    with open(log_file, "r") as f:
        lines = f.readlines()

        for line in lines:
            if "TOF::" not in line:  
                continue
            heights.append(int(line.split()[-1]))
            times.append(float(line.split()[1]))

    for i in range(1, len(times)):
        times[i] = times[i] - times[0]
    times[0] = 0

    plt.plot(times, heights)
    plt.xlabel("Time (s)")
    plt.ylabel("Height (cm)")
    plt.title("Tello Height")
    plt.show()