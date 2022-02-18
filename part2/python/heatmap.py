import matplotlib.pyplot as plt
import numpy as np
import yaml
from cpp import draw_nodes
from scipy.interpolate import griddata

# Read information from yaml file
with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/data.yaml", 'r') as stream:
  dataMap = yaml.safe_load(stream)

output = []
path = []
for obj in dataMap:
    #val = np.random.uniform(0, 3.3)
    val = obj['power']
    path.append(np.array([obj['position']['x'], obj['position']['y']]))
    output.append(np.array([obj['position']['x'], obj['position']['y'], val]))

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np


ngridx = 100
ngridy = 100

x = [point[0] for point in output]
y = [point[1] for point in output]
z = [point[2] for point in output]

#TODO PADDING FOR CONTOUR PERIMITER
"""
x += [-.8, -.8, .8, .8]
y += [.75, -.75, .75, -.75]
z += [0, 0, 0, 0]
"""

#fig, (ax1, ax2) = plt.subplots(nrows=2)
fig, ax1 = plt.subplots()

# -----------------------
# Interpolation on a grid
# -----------------------
# A contour plot of irregularly spaced data coordinates
# via interpolation on a grid.

# Create grid values first.
xi = np.linspace(-1.1, 1.1, ngridx)
yi = np.linspace(-1.1, 1.1, ngridy)

# Linearly interpolate the data (x, y) on a grid defined by (xi, yi).
# triang = tri.Triangulation(x, y)
# interpolator = tri.LinearTriInterpolator(triang, z)
# Xi, Yi = np.meshgrid(xi, yi)
# zi = interpolator(Xi, Yi)

# Note that scipy.interpolate provides means to interpolate data on a grid
# as well. The following would be an alternative to the four lines above:

zi = griddata((x, y), z, (xi[None, :], yi[:, None]), method='cubic')    # method = linear, quadratic, cubic ...

ax1.contour(xi, yi, zi, levels=10, linewidths=0.5, colors='k')
cntr1 = ax1.contourf(xi, yi, zi, levels=10, cmap="viridis")

fig.colorbar(cntr1, ax=ax1, label='Signal Power (dB)')
ax1.plot(x, y, 'r+', ms=3)
ax1.set(xlim=(-1, 1), ylim=(-1, 1))
ax1.set_title('RF Field Strength')
plt.xlabel("x (m) ")
plt.ylabel("y (m)")

"""
# ----------
# Tricontour
# ----------
# Directly supply the unordered, irregularly spaced coordinates
# to tricontour.

ax2.tricontour(x, y, z, levels=10, linewidths=0.5, colors='k')
cntr2 = ax2.tricontourf(x, y, z, levels=10, cmap="RdBu_r")

fig.colorbar(cntr2, ax=ax2)
ax2.plot(x, y, 'ko', ms=3)
ax2.set(xlim=(-1, 1), ylim=(-1, 1))
ax2.set_title('RF 2')

plt.subplots_adjust(hspace=0.5)


"""
plt.show()
