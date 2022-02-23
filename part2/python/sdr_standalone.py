from pylab import *
from rtlsdr import *
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import yaml

from go_to_specific_point_on_map import SLAM as slam
from sdr import*

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Write Information to YAML file all at once
def generate_rf_data(point_list):
      with open('/home/luis/catkin_ws/src/IIB_Project/part2/ros/samples.yaml', 'w') as f:
          index = 0
          for point in point_list:
            index += 1    
            print("- {filename: 'p%s', pose: %s, time: %s, power: %s, samples: %s}" % (index, point[0], point[1], point[2], point[3]), file = f)
            print("Sample Data Generated!")


def append_rf_data(point, index):
    
    index += 1  
    data_dict = "- {filename: 'p%s', pose: %s, time: %s, power: %s, samples: %s}" % (index, point[0], point[1], point[2], point[3])

    if os.path.isfile('/home/luis/catkin_ws/src/IIB_Project/part2/ros/samples.yaml') == True:
        with open('/home/luis/catkin_ws/src/IIB_Project/part2/ros/samples.yaml','r') as f:
            yamlfile = yaml.safe_load(f) # Note the safe_load
            yamlfile.update(data_dict)

        if yamlfile:
            with open('/home/luis/catkin_ws/src/IIB_Project/part2/ros/samples.yaml','w') as f:
                yaml.safe_dump(yamlfile, f) # Also note the safe_dump

        print("Sample Data Appended!")
    else:
        with open('/home/luis/catkin_ws/src/IIB_Project/part2/ros/samples.yaml', 'w') as f:  
            print(data_dict, file = f)
            print("Sample Data Created!")

# SDR attached to computer
# sdr = RtlSdr()
# SDR attached to Raspberry Pi
sdr = RtlSdrTcpClient(hostname='192.168.171.210', port=55366)
    
configure_device(sdr, center_freq=914.5e6)

data = []
localization = slam.SLAM()
index = 0

# Save position along with time in navigation script and sync both files later 

try:
    while True:
        # SLAM Localization
        localization.update()
        pose = np.array([localization.pose[X], localization.pose[Y], localization.pose[YAW]])
        
        # SDR Measurments
        samples = receive_samples(sdr)
        max_power, _ = get_power_from_PSD(samples, sdr, freq=915.1e6, plot=False)

        point = np.array([pose, time.time(), max_power, samples])
        append_rf_data(point, index)

        data.append(point)
        time.sleep(1)


except KeyboardInterrupt:
    print('Interrupted!')
    generate_rf_data(data)
