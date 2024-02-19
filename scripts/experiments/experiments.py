EXPERIMENTS = {}

"""
Show the construction time and storage for KDTree of 3D points, with increasing number of points
"""
EXPERIMENTS["ex_kdtree_construct"] = {
    "args": {
        "num_points": [2 ** i for i in range(1, 21)], 
        "num_experiments": [1 for _ in range(1, 11)] + [1 for _ in range(11, 21)]
    },
    "outputs": ["time"],
    "figures": [
        {
            "type": "bar+",
            "name": "time",
            "xcol": "num_points",
            "ycols": ["time"],

            "xlabel": "# of points",
            "ylabel": "Time [miliseconds]",
            "options": {
                "legend": {
                    "time": "Construction Time [miliseconds]"
                },
                "width": 25000
            }
        }
    ]
}

EXPERIMENTS["ex_kdtree_nn_omp"] = {
    "args": {
        "num_points": [2 ** i for i in range(1, 21)], 
        "num_queries": [2**20 for _ in range(1, 21)],
        "num_experiments": [1 for _ in range(1, 11)] + [1 for _ in range(11, 21)],
    },
    "outputs": ["serial_time", "parallel_time"],
    "figures": [
        {
            "type": "bar",
            "name": "time",
            "xcol": "num_points",
            "ycols": ["serial_time", "parallel_time"],
            "xlabel": "# of points",
            "ylabel": "Time [miliseconds]",
            "options": {
                "legend": {
                    "serial_time": "Serial NN",
                    "parallel_time": "Parallel NN [OpenMP]"
                },
                "width": 10000,
                # "yscale": "log",
                # "xscale": "log",
            }
        }
    ]
}

EXPERIMENTS["ex_kdtree_nn"] = {
    "args": {
        "num_points": [2 ** i for i in range(1, 21)], 
        "num_queries": [1024 * 1024 for _ in range(1, 21)],
        "num_experiments": [1 for _ in range(1, 11)] + [1 for _ in range(11, 21)],
    },
    "outputs": ["naive_time", "kdtreenn_time"],
    "figures": [
        {
            "type": "bar",
            "name": "time",
            "xcol": "num_points",
            "ycols": ["naive_time", "kdtreenn_time"],
            "xlabel": "# of points",
            "ylabel": "Time [miliseconds]",
            "options": {
                "legend": {
                    "naive_time": "Naive NN",
                    "kdtreenn_time": "KDTree NN"
                },
                "width": 10000,
                "yscale": "log",
                "xscale": "log",
            }
        }
    ]
}