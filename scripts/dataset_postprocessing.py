#!/usr/bin/env python

"""
Test script to read a CSV and analyze the raw data.
"""

import os
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

CSV_PATH = os.path.expanduser('~') + "/catkin_ws/src/proboscis_full/tension_calibration" + "/data/current_turns.csv"
N_MOTORS = 7
DISABLE_TORQUE_REQUEST = -1.0
MAX_TURN = 2.0
N_TRIAL = 20
POLY_ORDER = 2

def plot_csv(np_array, figure_id):
    # Only for Debug
    plt.figure(figure_id)

    for i in range(N_MOTORS):
        plt.plot(range(len(np_array)), np_array[:, i], label=f'Motor {i+1}')

    # Figure Properties
    plt.title('Currents')
    plt.xlabel('Indeces')
    plt.ylabel('Current [A]')
    plt.legend()
    plt.grid(True)

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
    derivatives = [0.0]

    for i in range(1, ds.shape[0]):
        derivatives.append((ds[i, 1] - ds[i - 1, 1])/(ds[i, 0] - ds[i - 1, 0]))
    return np.array(derivatives)

def main():
    dataset = genfromtxt(CSV_PATH, delimiter=',')
    # plot_csv(dataset, 1)

    # Extract Window and compute Mean
    for i in range(N_MOTORS):
        if not i == 4:
            # Select and Filter the Window
            filtered_dataset = find_window(dataset, i)

            # Create Figure
            plt.figure(2)
            plt.plot(filtered_dataset[:, 0], filtered_dataset[:, 1], label=f'Motor {i+1}', color='#0070c0')

            # Linear Regression
            x = filtered_dataset[:, 0].reshape((-1, 1))
            y = filtered_dataset[:, 1]
            model = LinearRegression().fit(x, y)

            plt.plot(filtered_dataset[:, 0], model.predict(x), label=f'Lin. Regression of Motor {i+1}', color='#e55934')

            poly_regr = PolynomialFeatures(degree = POLY_ORDER, include_bias=False)
            X_poly = poly_regr.fit_transform(x)
            model2 = LinearRegression().fit(X_poly, y)

            # print(model2.predict(X_poly))
            plt.plot(x, model2.predict(X_poly), label=f'Pol. Regression of Motor {i+1}', color='#72a98f')

            plt.figure(2)
            plt.title('Currents respect to Position')
            plt.xlabel('[n° turns]')
            plt.ylabel('[A]')
            plt.legend()
            plt.grid(True)
            plt.show()

        else:
            continue
    
    # plt.figure(2)
    # plt.title('Currents respect to Position')
    # plt.xlabel('[n° turns]')
    # plt.ylabel('[A]')
    # plt.legend()
    # plt.grid(True)
    # plt.show()


if __name__ == "__main__":
    main()