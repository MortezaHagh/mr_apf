import json
import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
# import seaborn as sns
from typing import Dict, List, Any

# Set up plotting style
plt.style.use('default')
# sns.set_palette("husl")
pd.set_option('display.max_columns', None)
pd.set_option('display.width', None)


def load_all_results(base_dir: str = '.') -> pd.DataFrame:
    results = []
    base_path = Path(base_dir)

    # Find all res.json files in subdirectories
    for json_file in base_path.rglob('res.json'):
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)

            # Add folder name for reference
            data['folder_name'] = json_file.parent.name
            results.append(data)
            # print(f"Loaded: {json_file}")

        except Exception as e:
            print(f"Error loading {json_file}: {e}")

    if not results:
        print("No res.json files found!")
        return pd.DataFrame()

    df = pd.DataFrame(results)
    return df


def refine_dataframe(df: pd.DataFrame, nd: str = '2D') -> pd.DataFrame:
    # Drop rows with nD == 2D 3D
    df = df[df['nD'] == nd]

    # Only keep relevant columns
    relevant_columns = ['map_id', 'n_robots', 'method', 'success', 'operation_time', 'total_length', 'max_steps']
    df = df[relevant_columns]

    return df


def map_tables(df: pd.DataFrame) -> Dict[str, pd.DataFrame]:
    # group by map_id and create tables for each map
    tables = {}
    for map_id, map_data in df.groupby('map_id'):
        tables[map_id] = map_data
    return tables


def create_performance_tables(tables: Dict[str, pd.DataFrame], metric: str) -> Dict[int, pd.DataFrame]:
    #  create table from df (which is for a specific map), columns are n_robots, index are methods, values are metric
    p_tables = {}
    for i, df in tables.items():
        table = df.pivot_table(index='method', columns='n_robots', values=metric, aggfunc=np.mean)
        p_tables[i] = table
    return p_tables


def get_performance_tables(path: str = './tests', nd='2D', metric: str = 'operation_time') -> Dict[int, pd.DataFrame]:
    df_all = load_all_results(path)
    df_all = refine_dataframe(df_all, nd=nd)

    # if success is False, set operation_time and total_length and max_steps to NaN
    df_all.loc[~df_all['success'], ['operation_time', 'total_length', 'max_steps']] = np.nan

    tables = map_tables(df_all)

    # Create performance tables
    p_tables = create_performance_tables(tables, metric)

    return p_tables

# def analyze_method_performance(df: pd.DataFrame):
#     """
#     Detailed analysis of method performance
#     """
#     print("=== METHOD PERFORMANCE ANALYSIS ===\n")

#     for map_id in sorted(df['map_id'].unique()):
#         map_data = df[df['map_id'] == map_id]
#         print(f"MAP {map_id} Analysis:")
#         print("-" * 30)

#         # Best method for each metric and robot count
#         metrics = ['operation_time', 'total_length']

#         for n_robots in sorted(map_data['n_robots'].unique()):
#             robot_data = map_data[map_data['n_robots'] == n_robots]
#             print(f"\n  {n_robots} Robots:")

#             for metric in metrics:
#                 if metric in robot_data.columns:
#                     best_method = robot_data.loc[robot_data[metric].idxmin(), 'method']
#                     best_value = robot_data[metric].min()
#                     worst_value = robot_data[metric].max()
#                     improvement = ((worst_value - best_value) / worst_value * 100)

#                     print(f"    {metric}: Method {best_method} (Best: {best_value:.2f}, "
#                           f"Improvement: {improvement:.1f}%)")

#         print("\n" + "="*50 + "\n")


# # Analyze method performance
# analyze_method_performance(df_all)
