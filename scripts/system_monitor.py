#!/usr/bin/env python3

import rospy
import traceback

from system_stats.nodes import NodeInfo
from system_stats.stats import SystemStats


class SystemMonitor(object):
    def __init__(self):
        try:
            self._pub_nodes = rospy.get_param("~pub_nodes")
            self._pub_dead = rospy.get_param("~pub_dead_nodes")
            self._separate_stats = rospy.get_param("~separate_stats")
        except KeyError:
            rospy.logfatal("SystemMonitor: error loading param")
            raise

        if self._pub_nodes:
            self._nodes = NodeInfo(self._pub_dead)

        self._stats = SystemStats(self._separate_stats)

    def run(self):
        if self._pub_nodes:
            try:
                self._nodes.update()
            except Exception as e:
                rospy.logerr("SystemMonitor: error updating nodes")
                rospy.logerr("SystemMonitor:\n {}".format(traceback.format_exc(e)))

        try:
            self._stats.update()
        except Exception as e:
            rospy.logerr("SystemMonitor: error updating system stats")
            rospy.logerr("SystemMonitor:\n {}".format(traceback.format_exc(e)))

if __name__ == "__main__":
    rospy.init_node("system_monitor")

    rate = rospy.Rate(10)
    node = SystemMonitor()

    while not rospy.is_shutdown():
        node.run()
        rate.sleep()
