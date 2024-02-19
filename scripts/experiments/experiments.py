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
            "ylabel": "Time [sec]",
            "options": {
                "legend": {
                    "time": "Construction Time [sec]"
                },
                "width": 25000
            }
        }
    ]
}

EXPERIMENTS["ex_kdtree_nn"] = {
    "args": {
        "num_points": [2 ** i for i in range(1, 21)], 
        "num_queries": [1024 * 16 for _ in range(1, 21)],
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
            "ylabel": "Time [sec]",
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