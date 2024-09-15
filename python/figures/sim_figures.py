import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

CUR_PATH = r'python\figures\results_140924.csv'

data = pd.read_csv(CUR_PATH)

def categorize_scene(environment):
    if 'lab363' in environment.lower():
        return 'Lab 363'
    elif 'lab446' in environment.lower():
        return 'Lab 446'
    elif 'perlin' in environment.lower():
        return 'Perlin'
    else:
        return 'ISPRS'

data['scene_category'] = data['environment'].apply(categorize_scene)

data['localizationVolumePercentage'] *= 100
data['localizationVolumePercentageXYT'] *= 100

scene_categories = ['Lab 363', 'Lab 446', 'Perlin', 'ISPRS']
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

unique_k_values = np.sort(data['k'].unique())

plt.figure(figsize=(10, 6))
bar_width = 0.2
x_values = np.arange(len(unique_k_values))

total_samples = len(data)

for i, scene in enumerate(scene_categories):
    scene_data = data[data['scene_category'] == scene]
    num_samples_scene = len(scene_data)
    grouped_data_k = scene_data.groupby('k')['localizationVolumePercentage'].sum()
    grouped_data_k = grouped_data_k.reindex(unique_k_values, fill_value=0)
    normalized_data_k = (grouped_data_k / num_samples_scene) * (num_samples_scene / total_samples)
    plt.bar(x_values + i * bar_width, normalized_data_k.values, width=bar_width, label=scene, color=colors[i])

plt.yscale('log')
plt.xlabel('k')
plt.ylabel('Normalized Average Localization Volume Percentage (%) [Log Scale]')
plt.title('Normalized Average Localization Volume Percentage Along k (by Scene)')
plt.xticks(x_values + bar_width, unique_k_values)
plt.legend()
plt.show()

plt.figure(figsize=(10, 6))

for i, scene in enumerate(scene_categories):
    scene_data = data[data['scene_category'] == scene]
    grouped_data_k = scene_data.groupby('k')['localizationVolumePercentage'].mean()
    grouped_data_k = grouped_data_k.reindex(unique_k_values, fill_value=0)
    plt.plot(unique_k_values, grouped_data_k.values, label=scene, color=colors[i], marker='o')

plt.xlabel('k')
plt.ylabel('Localization Volume Percentage (%)')
plt.title('Localization Volume Percentage Along k (by Scene)')
plt.xticks(unique_k_values)
plt.legend()
plt.show()

plt.figure(figsize=(10, 6))

for i, scene in enumerate(scene_categories):
    scene_data = data[data['scene_category'] == scene]
    time_data_k = scene_data.groupby('k')['time'].mean()
    time_data_k = time_data_k.reindex(unique_k_values, fill_value=0)
    plt.plot(unique_k_values, time_data_k.values, label=scene, color=colors[i], marker='o')

plt.xlabel('k')
plt.ylabel('Average Time Taken (seconds)')
plt.title('Runtime Comparison: Time Taken vs k (by Scene)')
plt.xticks(unique_k_values)
plt.legend()
plt.grid(True)
plt.show()

summary_table = pd.DataFrame(index=['Max Time (s)', 'Min Time (s)', 'Average Time (s)'])

for scene in scene_categories:
    scene_data = data[data['scene_category'] == scene]
    max_time = scene_data['time'].max()
    min_time = scene_data['time'].min()
    avg_time = scene_data['time'].mean()
    summary_table[scene] = [max_time, min_time, avg_time]

latex_code = summary_table.style.to_latex()

# with open('summary_table.tex', 'w') as f:
#     f.write(latex_code)

