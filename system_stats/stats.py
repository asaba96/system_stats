#!/usr/bin/env python3

# inspiration taken from
#    https://github.com/dheera/ros-system-stats/blob/master/system_stats/nodes/system_stats_node
# written by Andrew Saba

import psutil
import traceback

from system_stats.msg import Float32Stamped, Float32StampedArray, UInt64Stamped

from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def mean(list):
    return sum(list) / len(list)


class SystemStats(object):
    def __init__(self, sep_stats, node):
        self.sep_stats = sep_stats
        self._node = node

        self.network_ok = True
        try:
            interfaces_to_monitor = [
                k for k in psutil.net_io_counters(pernic=True, nowrap=True)
            ]

            self._node.get_logger().info(
                "SystemMonitor: monitoring interfaces \n{}".format(
                    interfaces_to_monitor
                )
            )
        except Exception as e:
            self._node.get_logger().error(
                "SystemMonitor: error getting network interfaces!"
            )
            self._node.get_logger().error(
                "SystemMonitor:\n {}".format(traceback.format_exc(e))
            )
            self.network_ok = False

        # if publish individual topics for each stat
        if sep_stats:
            self.cpu_temp_pub = node.create_publisher(
                Float32Stamped, "system/cpu/temp", 10
            )

            self.cpu_pub = node.create_publisher(Float32Stamped, "system/cpu/usage", 10)

            self.cpu_cores_pub = node.create_publisher(Float32StampedArray, "system/cpu/core/usage", 10)

            self.disk_pub = node.create_publisher(
                Float32Stamped, "system/disk/usage", 10
            )

            self.mem_pub = node.create_publisher(
                Float32Stamped, "system/mem/usage_virtual", 10
            )

            self.swap_pub = node.create_publisher(
                Float32Stamped, "system/mem/usage_swap", 10
            )

            self.interfaces = {}
            for interface in interfaces_to_monitor:
                rec_bytes_pub = node.create_publisher(
                    UInt64Stamped,
                    "system/network/{}/receive_bytes".format(interface.replace(".", "_").replace("-", "_")),
                    10,
                )

                send_bytes_pub = node.create_publisher(
                    UInt64Stamped, "system/network/{}/sent_bytes".format(interface.replace(".", "_").replace("-", "_")), 10
                )
                self.interfaces[interface] = (rec_bytes_pub, send_bytes_pub)

        self.stat_pub = node.create_publisher(DiagnosticArray, "system/diagnostics", 10)

        self.p = psutil.Process()

    def update(self):
        status_cpu = DiagnosticStatus()
        status_cpu.name = "CPU"
        status_cpu_cores = DiagnosticStatus()
        status_cpu_cores.name = "CPU Per Core"
        status_mem = DiagnosticStatus()
        status_mem.name = "Memory"
        status_disk = DiagnosticStatus()
        status_disk.name = "Disk"
        status_network = DiagnosticStatus()
        status_network.name = "Network"

        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()

        if self.sep_stats:
            cpu_temp = Float32Stamped()
            cpu_temp.header = header
            cpu = Float32Stamped()
            cpu.header = header
            cpu_core = Float32StampedArray()
            cpu_core.header = header
            disk = Float32Stamped()
            disk.header = header
            mem = Float32Stamped()
            mem.header = header
            swap = Float32Stamped()
            swap.header = header
            send_bytes = UInt64Stamped()
            send_bytes.header = header
            rec_bytes = UInt64Stamped()
            rec_bytes.header = header

        # one shot supposedly is faster since it uses cached values
        with self.p.oneshot():
            # cpu temp
            temps = psutil.sensors_temperatures()
            cpu_coretemp = None
            if "coretemp" in temps:
                cpu_coretemp = mean(list(map(lambda x: x.current, temps["coretemp"])))
                status_cpu.values.append(
                    KeyValue(key="coretemp", value=str(cpu_coretemp))
                )

            # cpu usage
            all_cores = psutil.cpu_percent(percpu=True)
            cpu_usage = mean(all_cores)
            status_cpu.values.append(KeyValue(key="usage", value=str(cpu_usage)))
            status_cpu_cores.values.append(KeyValue(key="per core usage", value=str(all_cores)))

            # disk
            disk_usage = psutil.disk_usage("/").percent
            status_disk.values.append(KeyValue(key="usage", value=str(disk_usage)))

            # memory
            mem_usage_virtual = psutil.virtual_memory().percent
            mem_usage_swap = psutil.swap_memory().percent
            status_mem.values.append(
                KeyValue(key="usage_virtual", value=str(mem_usage_virtual))
            )
            status_mem.values.append(
                KeyValue(key="usage_swap", value=str(mem_usage_swap))
            )

            # network
            if self.network_ok:
                for interface, (rec_pub, send_pub) in self.interfaces.items():
                    try:
                        net_stat = psutil.net_io_counters(pernic=True, nowrap=True)[
                            interface
                        ]
                        net_rec = int(net_stat.bytes_recv)
                        net_sent = int(net_stat.bytes_sent)
                    except Exception as e:
                        self._node.get_logger().error(
                            "SystemMonitor: error updating network stats for interface {}".format(
                                interface
                            )
                        )
                        self._node.get_logger().error(
                            "SystemMonitor:\n {}".format(traceback.format_exc(e))
                        )
                        self.network_ok = False
                    else:
                        status_network.values.append(
                            KeyValue(
                                key="{}/sent_bytes".format(interface),
                                value=str(net_sent),
                            )
                        )
                        status_network.values.append(
                            KeyValue(
                                key="{}/receive_bytes".format(interface),
                                value=str(net_rec),
                            )
                        )

                        if self.sep_stats:
                            rec_bytes.data = net_rec
                            send_bytes.data = net_sent
                            send_pub.publish(send_bytes)
                            rec_pub.publish(rec_bytes)

            if self.sep_stats:
                if cpu_coretemp is not None:
                    cpu_temp.data = cpu_coretemp
                cpu.data = cpu_usage
                cpu_core.data = all_cores
                disk.data = disk_usage
                mem.data = mem_usage_virtual
                swap.data = mem_usage_swap

                # publish
                self.cpu_temp_pub.publish(cpu_temp)
                self.cpu_pub.publish(cpu)
                self.cpu_cores_pub.publish(cpu_core)
                self.disk_pub.publish(disk)
                self.mem_pub.publish(mem)
                self.swap_pub.publish(swap)

        # all stats
        msg = DiagnosticArray()
        msg.status = [status_cpu, status_mem, status_disk, status_network]
        msg.header = header

        self.stat_pub.publish(msg)
