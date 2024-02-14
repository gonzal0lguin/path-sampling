#!/usr/bin/env python3

import rospy
from path_sampling_ros import PathSamplerROS

if __name__ == '__main__':
    rospy.init_node('path_sampling_node')

    ps = PathSamplerROS()

    rospy.spin()