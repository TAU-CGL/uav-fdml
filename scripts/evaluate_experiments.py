import os
import json
import itertools
import subprocess

import tqdm
import pandas as pd

from utils import *

SKIP_EXISTING = True

# EPS_VALUES = (0.001, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03)
# EPS_VALUES = (0.005, 0.015, 0.03, 0.05)
EPS_VALUES = (0.005, 0.01, 0.015)
ENVIRONMENT = "/fdml/scans/labs/lab446a.ply"
MEASUREMENTS = "/fdml/experiments/exp_mr_lh_446a.json"
EXPERIMENT_NAME = "experiment_accuracy"
RESULTS_DIR = "results"
DB_PATH = os.path.join(RESULTS_DIR, "results.db")


def run_experiment(epsilon):
    result = subprocess.run([
        get_executable_path(EXPERIMENT_NAME),
        "--epsilon", str(epsilon),
        "--environment", ENVIRONMENT,
        "--measurements", MEASUREMENTS
    ], cwd=get_bin_path(), capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr)
        return None
    output_data = json.loads(result.stdout)
    return pd.DataFrame(output_data)
    
if __name__ == "__main__":
    db = DB(DB_PATH)
    for epsilon in tqdm.tqdm(EPS_VALUES):
        print(epsilon)
        df = run_experiment(epsilon)
        if df is not None:
            df.to_sql("results", db.conn, if_exists="append", index=False)
    db.conn.close()