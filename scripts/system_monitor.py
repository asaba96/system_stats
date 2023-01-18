#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import traceback

from system_stats_library.stats import SystemStats


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
        self.declare_parameter("separate_stats", True)
        try:
            self._separate_stats = self.get_parameter("separate_stats")
        except KeyError:
            self.get_logger().error("SystemMonitor: error loading param")
            raise

        self._stats = SystemStats(self._separate_stats, self)

        # update at 5hz
        self.timer = self.create_timer(0.2, self.run)

    def run(self):
        try:
            self._stats.update()
        except Exception as e:
            self.get_logger().error("SystemMonitor: error updating system stats")
            self.get_logger().error(
                "SystemMonitor:\n {}".format(traceback.format_exc(e))
            )


def main(args=None):
    rclpy.init(args=args)

    node_ = SystemMonitor()
    rclpy.spin(node_)

    node_.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
