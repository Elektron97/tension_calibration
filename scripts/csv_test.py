#!/usr/bin/env python
import os
import csv
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

current_position_data = pd.read_csv(os.path.expanduser('~') + "/catkin_ws/src/proboscis_full/tension_calibration/data/current_turns_sample_copy.csv")

# Number of samples
n_samples = current_position_data.shape[0]

# Define Matrix that collects the derivatives
derivatives_matrix = np.zeros((n_samples - 1, 1))

# Compute Derivative
for i in range(n_samples - 1):
    current_increment =  (current_position_data.loc[i + 1, 'Current1'] - current_position_data.loc[i, 'Current1'])
    turns_increment = ((current_position_data.loc[i + 1, 'Turns1'] - current_position_data.loc[i, 'Turns1']))

    if turns_increment == 0:
        derivatives_matrix[i, 0] = np.nan
    else:
        derivatives_matrix[i, 0] = current_increment/turns_increment

print(derivatives_matrix[:, 0])
fig, ax = plt.subplots()
ax.plot(range(n_samples - 1), derivatives_matrix[:, 0], linewidth=2.0)
plt.grid()
plt.show()