import os
import itertools
import subprocess

import tqdm
import pandas as pd

from utils import *

SKIP_EXISTING = False

EXPERIMENT_ARGS = {
    "environment": [env.replace("resources", "") for env in get_available_environments("resources/fdml/scans/")],
    "k": list(range(4, 51, 2)),
    "delta": [0.1, 0.05, 0.025, 0.01],
    "epsilon": [0.0, 0.005, 0.01, 0.02, 0.05, 0.1],
    "num_experiments": [10],
}
EXPERIMENT_NAME = "experiment_accuracy"
RESULTS_DIR = "results"
DB_PATH = os.path.join(RESULTS_DIR, "results.db")

def init_table(db: DB):
    inner_query = ""
    for arg in EXPERIMENT_ARGS:
        argType = type(EXPERIMENT_ARGS[arg][0])
        if argType == str:
            inner_query += f"\t{arg} TEXT, "
        elif argType == int:
            inner_query += f"\t{arg} INTEGER, "
        elif argType == float:
            inner_query += f"\t{arg} REAL, "
        inner_query += "\n\t"
    inner_query = inner_query + f"\tUNIQUE({', '.join(EXPERIMENT_ARGS.keys())})"

    query = f"""
        CREATE TABLE IF NOT EXISTS results (
            id INTEGER PRIMARY KEY,
            {inner_query}
        )
    """
    db.cursor.execute(query)
    db.conn.commit

if __name__ == "__main__":
    db = DB(DB_PATH)
    init_table(db)

    combinations = list(itertools.product(*EXPERIMENT_ARGS.values()))
    arg_names = list(EXPERIMENT_ARGS.keys())

    for combo in tqdm.tqdm(combinations):
        if SKIP_EXISTING:
            query = f"SELECT * FROM results WHERE {' AND '.join([f'{arg} = ?' for arg in arg_names])}"
            if db.cursor.execute(query, combo).fetchone() is not None:
                continue

        # Run the experiment
        args = dict(zip(arg_names, combo))
        params = []
        for k, v in args.items():
            params.append(f"--{k}")
            params.append(str(v))
        output = subprocess.check_output([get_executable_path(EXPERIMENT_NAME)] + params, cwd=get_bin_path()).decode("utf-8")
        
        # Parse the results
        results = {}
        for line in output.splitlines():
            if ":" not in line:
                continue
            column, value = line.split(":", 1)
            results[column.strip()] = float(value.strip())
        results.update(args)

        existing_columns = [col[1] for col in db.cursor.execute(f"PRAGMA table_info(results)").fetchall()]
        new_columns = [col for col in results.keys() if col not in existing_columns]
        for col in new_columns:
            print(f"Warning: New column detected: {col}")
            db.cursor.execute(f"ALTER TABLE results ADD COLUMN {col} REAL")
            db.conn.commit()
        
        # Insert the results
        columns = ', '.join(results.keys())
        placeholders = ', '.join(['?'] * len(results))
        values = tuple(results.values())
        db.cursor.execute(f"INSERT OR REPLACE INTO results ({columns}) VALUES ({placeholders})", values)
        db.conn.commit()

    db.conn.close()
