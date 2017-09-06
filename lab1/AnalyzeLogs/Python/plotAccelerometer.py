from __future__ import print_function, division
# An example script you can use to plot data using python.

import numpy as np
import matplotlib.pyplot as plt

# The file name from which we get the data. You must, obviously, update this to 
# whatever your lof file of interest is called.
fname = 'log.csv'

#now we load the CSV file
data = np.genfromtxt(fname, delimiter=',', skip_header=1)
#parse the CSV data into fields we can use easily:
t = data[:,0]
accelerometer = data[:,1:4]
rateGyro = data[:,4:7]
heightSensor = data[:,7]
opticalFlow = data[:,8:10]
battVoltage = data[:,10]
motorCommands = data[:,11:15]
debugValues = data[:,15:27]
motorsOn = data[:,27]

# Change the below to whatever you want to do: 
# Here, we plot the accelerometer and some of the debug values against time.

fig = plt.figure()
n = 2  # num sub-plots
fig.add_subplot(n, 1, 1)
for i in range(2, n + 1):
    fig.add_subplot(n, 1, i, sharex=fig.axes[0])

fig.axes[0].plot(t, accelerometer[:,0], 'b', label='x')
fig.axes[0].plot(t, accelerometer[:,1], 'r', label='y')
fig.axes[0].plot(t, accelerometer[:,2], 'g', label='z')
fig.axes[0].set_ylabel('Acc meas [m/s^2]')
fig.axes[0].legend()

fig.axes[1].plot(t, debugValues[:,0], 'b', label='debug 0')
fig.axes[1].set_ylabel('Debug values')
fig.axes[1].legend()

fig.axes[1].set_xlabel('Time [s]')

plt.show()
