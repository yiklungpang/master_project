""" Scatter plot of ending position of affordance experiments
This program reads the affordance experiment results and plot
the ending positions of the objects
"""
# Author: Yik Lung Pang

import matplotlib.pyplot as plt
import numpy as np
import os
import json




# Open data directory
script_dir = os.path.dirname(os.path.realpath(__file__))
experiment_data_directory_path = os.path.join(script_dir, 'data/')
# Process all data files
for filename in os.listdir(experiment_data_directory_path):
    experiment_data_file_path = os.path.join(experiment_data_directory_path, filename)
    with open(experiment_data_file_path, 'r') as experiment_data_json:
        experiment_data = json.load(experiment_data_json)
        tool_name = experiment_data['tools']
        print("Showing "+tool_name)

        # Arrays to save the ending positions
        x_cube = np.array([])
        y_cube = np.array([])
        x_sphere = np.array([])
        y_sphere = np.array([])
        x_cylinder = np.array([])
        y_cylinder = np.array([])
        # Go through results for all objects
        for object_name in experiment_data['objects']:
            for result in experiment_data['objects'][object_name]['results']:
                action = result['action']
                # if action != 'tap_from_right':
                #   continue

                # Object bounced off work table, discard
                if result['end_pos']['z'] < 0.5:
                    print("WARNING: Invalid result. Discarded")
                    print(tool_name, object_name, action)
                    continue

                # Displacement           
                dx = result['end_pos']['x'] - result['start_pos']['x']
                dy = result['end_pos']['y'] - result['start_pos']['y']

                # Add to list for display
                if object_name == 'cube':
                    x_cube = np.append(x_cube, dx)
                    y_cube = np.append(y_cube, dy)
                elif object_name == 'sphere':
                    x_sphere = np.append(x_sphere, dx)
                    y_sphere = np.append(y_sphere, dy)
                elif object_name == 'cylinder':
                    x_cylinder = np.append(x_cylinder, dx)
                    y_cylinder = np.append(y_cylinder, dy)

        # Display scatter
        plt.scatter(x_cube, y_cube, marker='x', s=50,  c='r')
        plt.scatter(x_sphere, y_sphere, marker='x', s=50,  c='b')
        plt.scatter(x_cylinder, y_cylinder, marker='x', s=50,  c='g')
        plt.axhline(y=0.04, color='g', linestyle='--')
        plt.axhline(y=-0.04, color='g', linestyle='--')
        plt.axhline(y=0.08, color='r', linestyle='--')
        plt.axhline(y=-0.08, color='r', linestyle='--')
        plt.axvline(x=0.04, color='g', linestyle='--')
        plt.axvline(x=-0.04, color='g', linestyle='--')
        plt.axvline(x=0.08, color='r', linestyle='--')
        plt.axvline(x=-0.08, color='r', linestyle='--')

        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.xlim(0.3, -0.3)
        plt.ylim(0.3, -0.3)
        plt.grid()
        plt.title(tool_name)
        plt.show()

