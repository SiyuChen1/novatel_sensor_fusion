import pandas as pd
import matplotlib.pyplot as plt

# Load the csv files
raw_diff = pd.read_csv('raw_diff_uniklinikum.csv')
fused_diff = pd.read_csv('fused_diff_uniklinikum.csv')

# Display the first few rows of each dataset
raw_diff.head(), fused_diff.head()

# Rename columns for clarity
columns = ['Time', 'East', 'North', 'Up']
raw_diff.columns = columns
fused_diff.columns = columns

# Plot differences against time
directions = ['East', 'North', 'Up']

# Adjusted plotting code to avoid multi-dimensional indexing issues

fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 16))

for i, direction in enumerate(directions):
    axes[i].plot(raw_diff['Time'].values, raw_diff[direction].values, label='Raw', color='b')
    axes[i].plot(fused_diff['Time'].values, fused_diff[direction].values, label='Fused', color='r')
    axes[i].set_title(f'{direction} vs. Time')
    axes[i].set_ylabel(f'{direction} Difference in m')
    axes[i].set_xlim(left=0)  # Set the left limit to zero
    axes[i].legend()

axes[i].set_xlabel('Time')

plt.tight_layout()
plt.show()