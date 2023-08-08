from turtle import color
import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import yaml

from scipy.interpolate import griddata
from sklearn.metrics import r2_score

from occupancy_grid import*
import cpp
import sdr

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
  
  pluto_location = np.array([-0.75, 0.90])
  
  #TODO post-process data from RF samples in sample.yaml

  # Read information from yaml files

  # RF DATA
  #with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/data.yaml", 'r') as stream:
  with open("C:\\Users\\34606\\OneDrive - University of Cambridge\\Escritorio\\IIB_Project\\part2\\ros\\data.yaml", 'r') as stream:
    dataMap = yaml.safe_load(stream)

  # Process information from yaml file
  output = []
  sdr_poses = []
  for obj in dataMap:
    power = obj['power']
    sdr_poses.append(np.array([obj['position']['x'], obj['position']['y']]))
    output.append(np.array([obj['position']['x'], obj['position']['y'], power]))

  # PATH DATA
  #with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/path.yaml", 'r') as stream2:
  with open("C:\\Users\\34606\\OneDrive - University of Cambridge\\Escritorio\\IIB_Project\\part2\\ros\\path.yaml", 'r') as stream2:
    PathMap = yaml.safe_load(stream2)

  # Process information from yaml file
  path = []
  # spacing = np.arange(0, 19180, 40)

  for obj in PathMap:
    # if int((obj['filename'])[1:]) in spacing:
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
  zi = griddata((x, y), z, (xi[None, :], yi[:, None]), method='linear')    # method = linear, nearest, cubic ...

  ax1.contour(xi, yi, zi, levels=10, linewidths=0.5, colors='k')
  cntr1 = ax1.contourf(xi, yi, zi, levels=10, cmap="viridis")

  fig1.colorbar(cntr1, ax=ax1, label='Signal Power (dB)')
  ax1.plot(x, y, 'r+', ms=3)
  ax1.set(xlim=(-1, 1), ylim=(-1, 1))
  ax1.set_title('RF Field Strength')
  plt.xlabel("x (m) ")
  plt.ylabel("y (m)")
  
  # Calculate and draw Area Coverage
  
  percentage = cpp.area_covered(occupancy_grid, path, robot_radius=0.09, duplicates=True)
  plot_txt = "{}% Coverage".format(np.around(percentage, 1))
  
  fig2, ax2 = plt.subplots()

  occupancy_grid.draw_trajectory()
  # occupancy_grid.draw()
  cpp.draw_nodes(sdr_poses, 'green', size=6)
  cpp.draw_connections(path, linewidth=0.5, head_width=0.01, head_length=0.02, arrow_length=0.01)
  cpp.draw_individual_node(sdr_poses[0], 'violet', size=6)
  cpp.draw_individual_node(pluto_location, 'orange', size=20)

  plt.axis('equal')
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')
  ax2.set(xlim=(-1, 1), ylim=(-1, 1))
  ax2.set_title(str(plot_txt))
  # plt.text(-0.25, -0.9, plot_txt)

  # Plot Signal power vs range

  range = []    # x
  power = []    # y

  for point in output:
    dist = cpp.find_dist_from_a_to_b(pluto_location, point[:2])
    range.append(dist)
    power.append(point[2])

  x, y = range, power
  fig3, ax3 = plt.subplots()

  plt.scatter(x, y)
  plt.xlabel('Distance from Tx (m)')
  plt.ylabel('Signal Power (dB)')

  z = np.polyfit(x, y, 1)
  y_hat = np.poly1d(z)(x)
  plt.plot(x, y_hat, "r--")

  text = f"$y={z[0]:0.3f}\;x{z[1]:+0.3f}$\n$R^2 = {r2_score(y,y_hat):0.3f}$"
  plt.gca().text(0.05, 0.95, text,transform=plt.gca().transAxes, fontsize=14, verticalalignment='top')


  plt.show()

