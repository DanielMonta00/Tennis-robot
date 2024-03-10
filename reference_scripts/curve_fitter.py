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

# Time vector
nsamples = 50

t= np.linspace(-10,10, nsamples)
print(t)
params = [1, 0, -1, 0, -1, 0, 100]
#params =   [2.,   -3.9 , -2. ,   3.9 ,-4.2 , 15.8, -12. ]

# Parabola function
def parabola(t, a, b, c, d, e, h, i):
    x = a*t + b
    y = c*t + d
    z = e*t**2 + h*t + i
    return x, y, z

def xParab(t, a, b):
    x = a*t + b
    return x

def yParab(t,c, d):
    y = c*t + d
    return y

def zParab(t, e, h, i):
    z = e*t**2 + h*t + i
    return z

# Compute the parabola
x, y, z = parabola(t, *params)

#add gaussian noise

x+=np.random.normal(0, 0.01, size=nsamples) 
y+=np.random.normal(0, 0.01, size=nsamples) 
z+=np.random.normal(0, 0.01, size=nsamples) 

# Create 3D plot of the data points and the fitted curve 
fig = plt.figure() 
ax = fig.add_subplot(111, projection='3d') 
ax.scatter(x, y, z, color='blue') 

buffer_size = 50

# Fit the data to the parabola
poptx, pcovx = curve_fit(xParab, t[:buffer_size],x[:buffer_size]) 
popty, pcovy = curve_fit(yParab, t[:buffer_size],y[:buffer_size]) 
poptz, pcovz = curve_fit(zParab, t[:buffer_size],z[:buffer_size]) 

# Print optimized parameters, put them in a list

poptx = poptx.tolist()
popty = popty.tolist()
poptz = poptz.tolist()
estimated_params = np.array(poptx + popty + poptz)
print("estimated params")
print(estimated_params.round(1))     
print("real params")
print(params) 

# Plot the fitted curve

x_fit,y_fit,zfit = parabola(t,*estimated_params)
ax.plot(x_fit,y_fit, zfit, color='red')

# x_range = np.linspace(0, 1, 50) 
# y_range = np.linspace(0, 1, 50) 
# X, Y = np.meshgrid(x_range, y_range) 
# Z = func((X, Y), *popt) 
# ax.plot_surface(X, Y, Z, color='red', alpha=0.5) 
# ax.set_xlabel('X') 
# ax.set_ylabel('Y') 
# ax.set_zlabel('Z') 

plt.show()


