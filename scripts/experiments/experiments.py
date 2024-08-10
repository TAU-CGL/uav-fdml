EXPERIMENTS = {}

EXPERIMENTS["experiment_accuracy"] = {
    "args": {
        "num_experiments": [1 for _ in range(3, 16)],
        "k": list(range(3, 16)),
    },
    "outputs": ["accuracy", "success_rate", "time"],
    "figures": [
        {
            "type": "bar",
            "name": "success_rate",
            "xcol": "k",
            "ycols": ["success_rate"],
            "xlabel": "# odometries",
            "ylabel": "Success Rate [%]",
            "options": {
                "legend": {
                    "success_rate": "Success Rate",
                },
                "width": 0.5,
            }
            # "options": {
            #     "legend": {
            #         "success_rate": "Success Rate",
            #     },
            #     "width": 10000,
            #     "yscale": "log",
            #     "xscale": "log",
            # }
        }
    ]
}
