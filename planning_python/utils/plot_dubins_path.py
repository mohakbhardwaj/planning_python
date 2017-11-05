#!/usr/bin/env python

import dubins
import matplotlib.pyplot as plt
import numpy as np
import pprint

pp = pprint.PrettyPrinter(indent=4)

a = (0, 0, 0)
b = (0.5, 0.5, np.pi/4)
turning_radius = 1.0
step_size = turning_radius/10.0

path, _ = dubins.path_sample(a, b, turning_radius, step_size)

pp.pprint(path)

x = []
y = []
for state in path:
  x.append(state[0])
  y.append(state[1])

plt.plot(x,y)
plt.show()