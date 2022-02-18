#!/usr/bin/env python

import os
import sys
import numpy as np
import signal
import rospy
import yaml

from pylab import *
from rtlsdr import *

# Import code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
    import go_to_specific_point_on_map as Nav
    import measure_coverage as Area
    import sdr as SDR
  
except ImportError:
  raise ImportError('Unable to import functions. Make sure this file is in "{}"'.format(directory))

# Write Information to YAML file
def generate_rf_data(point_list):
      with open('data.yaml', 'w') as f:
          index = 0
          for point in point_list:
            index += 1    
            print("- {filename: 'p%s', position: { x: %s, y: %s}, power: %s}" % (index, point[0], point[1], point[2]), file = f)
            print(" RF Data File Generated")
      
def handler(signum, frmae):
    raise Exception("Unable to reach pose :(")

if __name__ == '__main__':

    print("follow_route running")
    rospy.loginfo("follow_route running")
    # Read information from yaml file
    with open("/home/luis/catkin_ws/src/IIB_Project/part2/python/route.yaml", 'r') as stream:
        dataMap = yaml.safe_load(stream)

    print("loaded")

    # m = Area.MeasureAreaCovered()

    try:
        # Initialize
        rospy.init_node('follow_route', anonymous=False)
        navigator = Nav.GoToPose()
        #slam = Nav.SLAM()

        sdr = RtlSdrTcpClient(hostname='192.168.229.210', port=55366)
        SDR.configure_device(sdr)
        
        data = []

        for obj in dataMap:

            if rospy.is_shutdown():
                break
            name = obj['filename']
            print(obj)
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(35)
            # Navigation
            try:
                rospy.loginfo("Go to %s pose", name[:-4])
                navigator.goto(obj['position'], obj['rotation'])
                #rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Reached %s pose", name[:-4])

                # Measure and calculate RF signal power
                samples = SDR.receive_samples(sdr)
                max_power, _ = SDR.calc_max_power(samples)
                max_power_dB = SDR.dB(max_power)

                data.append(np.array([obj['position']['x'], obj['position']['y'], max_power_dB]))

            except Exception as exc:
                print(exc)

        generate_rf_data(data)
        # m.disp()


    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
        generate_rf_data(data)