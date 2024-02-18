import os
import subprocess
from typing import List, Tuple, Dict

import pandas as pd
import matplotlib.pyplot as plt

from experiments import *


BIN_DIR = "bin"
RESULTS_DIR = "results"


# TODO: Add some form of caching
def _get_instance_list(args: dict):
    """
    From a single args dict, split to array of single instances
    """
    if len(args) == 0:
        raise ValueError("some args should be passed")
    # TODO: Consider doing this as a matrix
    num_instances = len(args[list(args.keys())[0]])
    instances = []
    for i in range(num_instances):
        instance = {}
        for arg in args:
            if len(args[arg]) != num_instances:
                raise ValueError("all args should be of the same length")
            instance[arg] = args[arg][i]
        instances.append(instance)
    return instances

def _get_project_source_dir():
    return __file__.split("scripts")[0]

def _get_executable_name(name: str):
    project_source_dir = _get_project_source_dir()
    executable = os.path.join(project_source_dir, BIN_DIR, name)
    if os.name == "nt":
        executable += ".exe"
    return executable

def _run_instance(instance: dict, executable: str):
    params = [executable]
    for arg in instance:
        params.append(f"--{arg}")
        params.append(str(instance[arg]))
    return subprocess.check_output(params)

def _result_to_table(results: List[Tuple[Dict, str]], outputs: List[str]):
    d = {key: [] for key in results[0][0]}
    for label in outputs:
        d[label] = []

    for instance, output in results:
        for arg, value in instance.items():
            d[arg].append(value)
        output = output.strip()
        for idx, line in enumerate(output.splitlines()):
            d[outputs[idx]].append(float(line))
    
    return pd.DataFrame(d)

def _get_table_name(name: str):
    project_source_dir = _get_project_source_dir()
    return os.path.join(project_source_dir, RESULTS_DIR, name + ".csv")

def run_experiment(name: str, args: dict, outputs) -> List[Tuple[Dict, str]]:
    instances = _get_instance_list(args)
    executable = _get_executable_name(name)
    
    results = []
    for instance in instances:
        print("Running: ", instance)
        results.append((instance, _run_instance(instance, executable)))
        print("Done! ....")
        print()
    
    df = _result_to_table(results, outputs)
    df.to_csv(_get_table_name(name))

def create_figure(
        experiment_name: str, figure_type: str, figure_name: str, 
        x_col: str, labels: List[str],
        hor_label: str, ver_label: str, options: dict):
    df = pd.read_csv(_get_table_name(experiment_name))

    xs = df[x_col]
    for label in labels:
        if figure_type.startswith("bar"):
            plt.bar(xs, df[label], width=options["width"], label=options["legend"][label])
        if figure_type == "plot" or figure_type.endswith('+'):
            plt.plot(xs, df[label], 'k-')
            plt.plot(xs, df[label], 'ko')
    
    plt.xlabel(hor_label)
    plt.ylabel(ver_label)
    plt.legend()
    
    figure_name = os.path.join(_get_project_source_dir(), RESULTS_DIR, f"{experiment_name}_{figure_name}.png")
    plt.savefig(figure_name)
    plt.close()
    

if __name__ == "__main__":
    for experiment, d in EXPERIMENTS.items():
        run_experiment(experiment, d["args"], d["outputs"])
        for figure in d["figures"]:
            create_figure(experiment, 
                          figure["type"], figure["name"], figure["xcol"], figure["ycols"], 
                          figure["xlabel"], figure["ylabel"], figure["options"])