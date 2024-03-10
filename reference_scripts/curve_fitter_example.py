#!/usr/bin/env python 

# 3D curve fitting 
# Input data: 3D points (x, y, z) from OptiTrack
# To determine: size of a buffer to store the last n points, 
# fitting method, distance threshold for adding points to the buffer
# Output: Fitted curve in 3D space

# The equation of the parabola in space is given by:
# (x,y,z) = (at^2 + bt + c, dt^2 + et + f, gt^2 + ht + i)
# where t is the time, and a, b, c, d, e, f, g, h, i are the coefficients to be determined
# Parabolas with gravity in the z direction and constant speed on xy plane are given by:

# (x,y,z) = (at + c, bt + d, -0.5gt^2+ht+i)

import numpy as np 
from scipy.optimize import curve_fit 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D 

# Generate random 3D data points 
x = np.random.random(100) 
y = np.random.random(100) 
z = np.sin(x * y) + np.random.normal(0, 0.1, size=100) 
data = np.array([x, y, z]).T 

# t= np.linspace(-10, 10, 100)
# params = [1, 0, -1, 0, -1, 0, 3]

# # Parabola function
# def parabola(t, a, b, c, d, e, h, i):
#     x = a*t + b
#     y = c*t + d
#     z = e*t**2 + h*t + i
#     return x, y, z

# Compute the parabola
# x, y, z = parabola(t, *params)

# Define mathematical function for curve fitting 
def func(xy, a, b, c, d, e, f): 
	x, y = xy 
	return a + b*x + c*y + d*x**2 + e*y**2 + f*x*y 

# Perform curve fitting 
popt, pcov = curve_fit(func, (x, y), z) 

# Print optimized parameters 
print(popt) 

# Create 3D plot of the data points and the fitted curve 
fig = plt.figure() 
ax = fig.add_subplot(111, projection='3d') 
ax.scatter(x, y, z, color='blue') 
x_range = np.linspace(0, 1, 50) 
y_range = np.linspace(0, 1, 50) 
X, Y = np.meshgrid(x_range, y_range) 
Z = func((X, Y), *popt) 
ax.plot_surface(X, Y, Z, color='red', alpha=0.5) 
ax.set_xlabel('X') 
ax.set_ylabel('Y') 
ax.set_zlabel('Z') 
plt.show()

