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
    print(np_array[:, motor_id - 1])
    print(np_array[:, N_MOTORS + motor_id - 1])
    indeces = np_array[:, N_MOTORS + motor_id -1] != DISABLE_TORQUE_REQUEST
    print(indeces)
    test = np_array[indeces, motor_id -1]
    print(test)
    
    # plt.figure()
    # plt.plot(range(len(test)), test, label=f'Motor {1}')    
    # # Figure Properties
    # plt.title('Currents')
    # plt.xlabel('Indeces')
    # plt.ylabel('Current [A]')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

def main():
    dataset = genfromtxt(CSV_PATH, delimiter=',')
    find_window(dataset, 1)
    # plot_csv(dataset)

if __name__ == "__main__":
    main()