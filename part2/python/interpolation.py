import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import yaml
from math import pow, sqrt  

from scipy.interpolate import griddata
import pyKriging
from pyKriging.krige import kriging

# RF DATA
# with open("/home/luis/catkin_ws/src/IIB_Project/part2/ros/data.yaml", 'r') as stream:
with open("C:\\Users\\34606\\OneDrive - University of Cambridge\\Escritorio\\IIB_Project\\part2\\ros\\data.yaml", 'r') as stream:
    dataMap = yaml.safe_load(stream)

# Process information from yaml file
output = []
sdr_poses = []
for obj in dataMap:
    power = obj['power']
    sdr_poses.append(np.array([obj['position']['x'], obj['position']['y']]))
    output.append(np.array([obj['position']['x'], obj['position']['y'], power]))

points = np.array(sdr_poses)
values = np.array([point[2] for point in output])


def pointValue(x,y,power,smoothing,xv,yv,values):  
    nominator=0  
    denominator=0  
    for i in range(0,len(values)):  
        dist = sqrt((x-xv[i])*(x-xv[i])+(y-yv[i])*(y-yv[i])+smoothing*smoothing);  
        #If the point is really close to one of the data points, return the data point value to avoid singularities  
        if(dist<0.0000000001):  
            return values[i]  
        nominator=nominator+(values[i]/pow(dist,power))  
        denominator=denominator+(1/pow(dist,power))  
    #Return NODATA if the denominator is zero  
    if denominator > 0:  
        value = nominator/denominator  
    else:  
        value = -9999  
    return value  
  
def invDist(xv,yv,values,xsize=100,ysize=100,power=2,smoothing=0):  
    valuesGrid = np.zeros((ysize,xsize))  
    for x in range(0,xsize):  
        for y in range(0,ysize):  
            valuesGrid[y][x] = pointValue(x,y,power,smoothing,xv,yv,values)  
    return valuesGrid  
      
  
if __name__ == "__main__":

    """
    # First Approach

    grid_x, grid_y = np.mgrid[-1:1:300j, -1:1:300j]

    grid_z0 = griddata(points, values, (grid_x, grid_y), method='nearest')
    grid_z1 = griddata(points, values, (grid_x, grid_y), method='linear')
    grid_z2 = griddata(points, values, (grid_x, grid_y), method='cubic')

    # plt.subplot(1)
    # plt.imshow(func(grid_x, grid_y).T, extent=(0,1,0,1), origin='lower')
    # plt.plot(points[:,0], points[:,1], 'k.', ms=1)
    plt.title('Original')
    plt.subplot(222)
    plt.imshow(grid_z0.T, extent=(0,1,0,1), origin='lower')
    plt.title('Nearest')
    plt.subplot(223)
    plt.imshow(grid_z1.T, extent=(0,1,0,1), origin='lower')
    plt.title('Linear')
    plt.subplot(224)
    plt.imshow(grid_z2.T, extent=(0,1,0,1), origin='lower')
    #plt.plot(points[:][0], points[:][1], 'r+', ms=1)
    plt.title('Cubic')
    plt.gcf().set_size_inches(6, 6)
    plt.show()
    

    # Second Approach

    power=1  
    smoothing=20  
  
    #Creating some data, with each coodinate and the values stored in separated lists  
    # xv = [10,60,40,70,10,50,20,70,30,60]  
    # yv = [10,20,30,30,40,50,60,70,80,90]  
    # values = [1,2,2,3,4,6,7,7,8,10]
     
    xv = [point[0] for point in output]  
    yv = [point[1] for point in output]  
    
      
    #Creating the output grid (100x100, in the example)  
    ti = np.linspace(-1, 1, 100)  
    XI, YI = np.meshgrid(ti, ti)  
  
    #Creating the interpolation function and populating the output matrix value  
    ZI = invDist(xv,yv,values,100,100,power,smoothing)  
  
  
    # Plotting the result  
    n = plt.Normalize(0.0, 100.0)  
    plt.subplot(1, 1, 1)  
    plt.pcolor(XI, YI, ZI)  
    plt.scatter(xv, yv, 100, values)  
    plt.title('Inv dist interpolation - power: ' + str(power) + ' smoothing: ' + str(smoothing))  
    plt.xlim(-1, 1)  
    plt.ylim(-1, 1)  
    plt.colorbar() 

    plt.show() 

    """

    # Third Approach

    optimizer = 'pso'

    k = kriging(points, values)
    k.train(optimizer = optimizer)
    k.plot()
    