import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv(r'python\figures\results_v3.csv')

def categorize_scene(environment):
    if 'lab363' in environment.lower():
        return 'lab363'
    elif 'lab446' in environment.lower():
        return 'lab446'
    elif 'perlin' in environment.lower():
        return 'perlin'
    else:
        return 'isprs'

data['scene_category'] = data['environment'].apply(categorize_scene)
data['localizationVolumePercentage'] *= 100
data['localizationVolumePercentageXYT'] *= 100

scene_categories = ['lab363', 'lab446', 'perlin', 'isprs']

columns_to_calculate = ['localizationVolumePercentage', 'localizationVolumePercentageXYT', 'time']

for scene in scene_categories:
    print(f"\nStatistics for {scene}:")
    scene_data = data[data['scene_category'] == scene]
    
    for column in columns_to_calculate:
        max_value = scene_data[column].max()
        min_value = scene_data[column].min()
        avg_value = scene_data[column].mean()
        print(f"{column} - Max: {max_value:.2f}, Min: {min_value:.2f}, Avg: {avg_value:.2f}")

unique_k_values = np.sort(data['k'].unique())

plt.figure(figsize=(10, 6))
bar_width = 0.2
x_values = np.arange(len(unique_k_values))

total_samples = len(data)

for i, scene in enumerate(scene_categories):
    scene_data = data[data['scene_category'] == scene]
    num_samples_scene = len(scene_data)
    grouped_data_k = scene_data.groupby('k')['localizationVolumePercentageXYT'].sum()
    grouped_data_k = grouped_data_k.reindex(unique_k_values, fill_value=0)
    normalized_data_k = (grouped_data_k / num_samples_scene) * (num_samples_scene / total_samples)
    plt.bar(x_values + i * bar_width, normalized_data_k.values, width=bar_width, label=scene, color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'][i])

plt.yscale('log')
plt.xlabel('k')
plt.ylabel('Localization Volume PercentageXYT (%)')
plt.xticks(x_values + bar_width, unique_k_values)
plt.legend()
plt.show()

unique_epsilon_values = np.sort(data['epsilon'].unique())

plt.figure(figsize=(10, 6))
bar_width = 0.2
x_values = np.arange(len(unique_epsilon_values))

for i, scene in enumerate(scene_categories):
    scene_data = data[data['scene_category'] == scene]
    num_samples_scene = len(scene_data)
    grouped_data_epsilon = scene_data.groupby('epsilon')['localizationVolumePercentageXYT'].sum()
    grouped_data_epsilon = grouped_data_epsilon.reindex(unique_epsilon_values, fill_value=0)
    normalized_data_epsilon = (grouped_data_epsilon / num_samples_scene) * (num_samples_scene / total_samples)
    plt.bar(x_values + i * bar_width, normalized_data_epsilon.values, width=bar_width, label=scene, color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'][i])

plt.yscale('log')
plt.xlabel('$\epsilon$')
plt.ylabel('Localization Volume PercentageXYT (%)')
plt.xticks(x_values + bar_width, unique_epsilon_values)
plt.legend()
plt.show()
