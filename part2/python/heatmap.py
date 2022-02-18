import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import yaml

from scipy.interpolate import griddata

from occupancy_grid import*
import cpp

"""
# Read information from yaml file
with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/data.yaml", 'r') as stream:
  dataMap = yaml.safe_load(stream)

# Process information from yaml file
output = []
path = []
for obj in dataMap:
    power = obj['power']
    path.append(np.array([obj['position']['x'], obj['position']['y']]))
    output.append(np.array([obj['position']['x'], obj['position']['y'], power]))


ngridx = 100
ngridy = 100

x = [point[0] for point in output]
y = [point[1] for point in output]
z = [point[2] for point in output]

#TODO PADDING FOR CONTOUR PERIMITER

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

plt.show()
"""

if __name__ == '__main__':

  # Read information from yaml files

  # RF DATA
  #with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/data.yaml", 'r') as stream:
  with open("C:\\Users\\34606\\OneDrive - University of Cambridge\\Escritorio\\IIB_Project\\part2\\ros\\data.yaml", 'r') as stream:
    dataMap = yaml.safe_load(stream)

  # Process information from yaml file
  output = []
  for obj in dataMap:
    power = obj['power']
    output.append(np.array([obj['position']['x'], obj['position']['y'], power]))

  # PATH DATA
  #with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/path.yaml", 'r') as stream:
  with open("C:\\Users\\34606\\OneDrive - University of Cambridge\\Escritorio\\IIB_Project\\part2\\ros\\path.yaml", 'r') as stream2:
    PathMap = yaml.safe_load(stream2)

  # Process information from yaml file
  path = []
  for obj in PathMap:
    power = obj['power']
    path.append(np.array([obj['position']['x'], obj['position']['y']]))
    

  # Draw Contour Heatmap
  
  ngridx = 100
  ngridy = 100

  x = [point[0] for point in output]
  y = [point[1] for point in output]
  z = [point[2] for point in output]

  fig1, ax1 = plt.subplots()

  xi = np.linspace(-1.1, 1.1, ngridx)
  yi = np.linspace(-1.1, 1.1, ngridy)
  zi = griddata((x, y), z, (xi[None, :], yi[:, None]), method='cubic')    # method = linear, quadratic, cubic ...

  ax1.contour(xi, yi, zi, levels=10, linewidths=0.5, colors='k')
  cntr1 = ax1.contourf(xi, yi, zi, levels=10, cmap="viridis")

  fig1.colorbar(cntr1, ax=ax1, label='Signal Power (dB)')
  ax1.plot(x, y, 'r+', ms=3)
  ax1.set(xlim=(-1, 1), ylim=(-1, 1))
  ax1.set_title('RF Field Strength')
  plt.xlabel("x (m) ")
  plt.ylabel("y (m)")
  
  # Calculate and draw Area Coverage
  
  percentage = cpp.area_covered(occupancy_grid, path, robot_radius=ROBOT_RADIUS, duplicates=False)
  plot_txt = "{}% Coverage".format(np.around(percentage, 1))
  
  fig2, ax2 = plt.subplots()

  occupancy_grid.draw_trajectory()
  cpp.draw_nodes(path, 'green')
  cpp.draw_connections(path, head_width=0.02, head_length=0.04, arrow_length=0.03)
  cpp.draw_individual_node(path[0], 'violet')

  plt.axis('equal')
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')
  ax2.set(xlim=(-1, 1), ylim=(-1, 1))
  ax2.set_title('Area Coverage')
  plt.text(-0.25, -0.85, plot_txt)

  plt.show()

