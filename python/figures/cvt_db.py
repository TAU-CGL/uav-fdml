import sqlite3
import pandas as pd

if __name__ == "__main__":
    conn1 = sqlite3.connect("results/results_v3.db")
    conn2 = sqlite3.connect("results/results_v3_2.db") # More values of k
    df1 = pd.read_sql_query("SELECT * FROM results", conn1)
    df2 = pd.read_sql_query("SELECT * FROM results", conn2)
    df = pd.concat([df1, df2])
    df.to_csv("results/results_v3.csv", index=False)