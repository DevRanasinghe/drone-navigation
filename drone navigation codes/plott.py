#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

# Read data from the file
data0 = []
data1 = []
data2 = []

def read_data(filename):
    data = []
    with open(filename, "r") as data_file:
        for line in data_file:
            value = float(line.strip())  # Convert to float
            data.append(value)
    return data


data_file0 = "data0.txt"  # Replace with your actual file paths
data_file1 = "data1.txt"
data_file2 = "data2.txt"

data0 = read_data(data_file0)
data1 = read_data(data_file1)
data2 = read_data(data_file2)

x = np.arange(1, len(data0) + 1)
cs = CubicSpline(x, data0)

x_interp = np.linspace(min(x), max(x), 1000)  # Increase the number of points for a smoother curve
y_interp = cs(x_interp)

#window_size = 5
#smoothed_data = np.convolve(data2, np.ones(window_size)/window_size, mode='same')

plt.subplot(311)  # Three rows, one column, first subplot
plt.plot(data0, marker='o', label='Data I0')
y_pad = 1.0
y_min = min(data0) - y_pad
y_max = max(data0) + y_pad
plt.plot(x_interp, y_interp, label="Cubic Spline Interpolation", color='red')
plt.ylim(y_min,y_max)
plt.ylabel('Value')
plt.title('z0')
plt.legend()



plt.subplot(312)  # Three rows, one column, second subplot
plt.plot(data1, marker='o', label='Data I1')
y_pad = 1.0
y_min = min(data1) - y_pad
y_max = max(data1) + y_pad
plt.ylim(y_min,y_max)
plt.ylabel('Value')
plt.title('z1')
plt.legend()



plt.subplot(313)  # Three rows, one column, third subplot
plt.plot(data2, marker='o', label='Data 2')
y_pad = 1.0
y_min = min(data2) - y_pad
y_max = max(data2) + y_pad
plt.ylim(y_min,y_max)
plt.ylabel('Value')
plt.title('z2')
plt.legend()

plt.tight_layout()

plt.show()

'''x = np.array([1, 2, 3, 4, 5])  # Add your x-values here
y = np.array([3, 1, 4, 2, 5])  # Add your y-values here

# Create a cubic spline interpolation function
cs = CubicSpline(x, y)

# Predict new values
new_x = np.array([6, 7, 8, 9, 10])  # Add the x-values you want to predict
predicted_y = cs(new_x)

print("Predicted Y values:", predicted_y)

plt.plot(y, marker='x', label='Data 1')
y_pad = 1.0
y_min = min(y) - y_pad
y_max = max(y) + y_pad
plt.ylim(y_min,y_max)
plt.ylabel('Value')
plt.title('pitch')
plt.legend()
plt.show()'''