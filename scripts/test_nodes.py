#!/usr/bin/env python3

import rospy
from system_stats.nodes import NodeInfo

if __name__ == "__main__":
    rospy.init_node('testing')

    r = rospy.Rate(5)
    n = NodeInfo()

    while not rospy.is_shutdown():
        n.update()
        r.sleep()
