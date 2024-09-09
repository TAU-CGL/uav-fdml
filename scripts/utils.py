import os
from typing import List

import sqlite3


def get_available_environments(path: str) -> List[str]:
    if path.endswith(".obj"):
        return [path]
    elif os.path.isdir(path): # Recurse
        res = []
        for subpath in os.listdir(path):
            res += get_available_environments(os.path.join(path, subpath))
        return res
    else:
        return []
    
class DB(object):
    def __init__(self, path):
        self.conn = sqlite3.connect(path)
        self.cursor = self.conn.cursor()

def get_bin_path():
    if os.name == "nt":
        return os.path.join(__file__.split("scripts")[0], "bin", "Release")
    else:
        return os.path.join(__file__.split("scripts")[0], "bin")

def get_executable_path(name: str):
    executable_path = os.path.join(get_bin_path(), name)
    if os.name == "nt":
        executable_path += ".exe"
    return executable_path

