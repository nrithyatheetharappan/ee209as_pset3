
"""
oscillating_circle.py
A simple matplotlib animation example that shows an oscillating circle.
electronut.in
"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import math
import numpy as np

count = 0
def update(frameNum, a0):
    global count
    A = math.sin(math.radians(count))
    x = [A*math.cos(a) for a in np.arange(0, 2*math.pi+0.1, 0.1)]
    y = [math.sin(a) for a in np.arange(0, 2*math.pi+0.1, 0.1)]
    a0.set_data(x, y)
    count = (count + 1) % 360

fig = plt.figure()
ax = plt.axes(xlim=(-1, 1), ylim=(-1, 1))
a0, = ax.plot([], [])
anim = animation.FuncAnimation(fig, update,
                               fargs=(a0,),
                               interval=20)
# show plot
plt.show()