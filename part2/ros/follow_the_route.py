#!/usr/bin/env python

import os
import sys
import numpy as np
import signal
import rospy
import yaml

# Import code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
    import go_to_specific_point_on_map as Nav
    import measure_coverage as Area
  
except ImportError:
  raise ImportError('Unable to import functions. Make sure this file is in "{}"'.format(directory))


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

        for obj in dataMap:

            if rospy.is_shutdown():
                break
            name = obj['filename']
            print(obj)
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(15)
            # Navigation
            try:
                rospy.loginfo("Go to %s pose", name[:-4])
                navigator.goto(obj['position'], obj['rotation'])
                rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Reached %s pose", name[:-4])
            except Exception as exc:
                print(exc)


        # m.disp()


    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")