#!/usr/bin/env python

"""
Test script to read a CSV and analyze the raw data.
"""

import os
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

CSV_PATH = os.path.expanduser('~') + "/catkin_ws/src/proboscis_full/tension_calibration" + "/data/current_turns.csv"
N_MOTORS = 7
DISABLE_TORQUE_REQUEST = -1.0
MAX_TURN = 2.0
N_TRIAL = 20

def plot_csv(np_array):
    # Only for Debug
    plt.figure()

    for i in range(N_MOTORS):
        plt.plot(range(len(np_array)), np_array[:, i], label=f'Motor {i+1}')

    # Figure Properties
    plt.title('Currents')
    plt.xlabel('Indeces')
    plt.ylabel('Current [A]')
    plt.legend()
    plt.grid(True)
    plt.show()

def find_window(np_array, motor_id):
    # Find the window of the specific motor
    indeces = (np_array[:, N_MOTORS + motor_id] != DISABLE_TORQUE_REQUEST)

    # Extract the data from turns and current
    turns_window = np_array[indeces, N_MOTORS + motor_id]
    current_window = np_array[indeces, motor_id]

    turn_step = np.linspace(0.0, MAX_TURN, N_TRIAL + 1)

    filtered_dataset = []

    for step_value in turn_step:
        # Extract the samples for each step
        step_indeces = np.round(turns_window, 3) == np.round(step_value, 3)
        current_samples = current_window[step_indeces]

        # Filtered Dataset
        filtered_dataset.append([np.round(step_value, 3), np.mean(current_samples)])

    return np.array(filtered_dataset)

def compute_derivatives(ds):
    derivatives = []

    for i in range(ds.shape[0] - 1):
        derivatives.append((ds[i + 1, 1] - ds[i, 1])/(ds[i + 1, 0] - ds[i, 0]))
    return np.array(derivatives)

def main():
    dataset = genfromtxt(CSV_PATH, delimiter=',')

    # Extract Window and compute Mean
    for i in range(N_MOTORS):
        if not i == 4:
            derivatives = compute_derivatives(find_window(dataset, i))

            plt.figure()
            plt.plot(range(len(derivatives)), derivatives, label=f'Motor {i+1}') 
            plt.title('Derivative of Currents respect to Position')
            plt.xlabel('Indeces')
            plt.ylabel('[A]/[n° turns]')
            plt.legend()
            plt.grid(True)
            plt.show()
        else:
            continue

if __name__ == "__main__":
    main()