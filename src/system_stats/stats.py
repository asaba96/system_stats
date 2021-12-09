# inspiration taken from
#    https://github.com/dheera/ros-system-stats/blob/master/system_stats/nodes/system_stats_node
# written by Andrew Saba

import psutil
import rospy

from system_stats.msg import Float32Stamped

from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def mean(list):
    return sum(list)/len(list)


class SystemStats(object):
    def __init__(self, sep_stats):
        self.sep_stats = sep_stats

        # if publish individual topics for each stat
        if sep_stats:
            self.cpu_temp_pub  = rospy.Publisher(
                "system/cpu/temp",
                Float32Stamped,
                queue_size=10
            )

            self.cpu_pub = rospy.Publisher(
                "system/cpu/usage",
                Float32Stamped,
                queue_size=10
            )

            self.disk_pub = rospy.Publisher(
                "system/disk/usage",
                Float32Stamped,
                queue_size=10
            )

            self.mem_pub = rospy.Publisher(
                "system/mem/usage_virtual",
                Float32Stamped,
                queue_size=10
            )

            self.swap_pub = rospy.Publisher(
                "system/mem/usage_swap",
                Float32Stamped,
                queue_size=10
            )


        self.stat_pub = rospy.Publisher(
            "system/diagnostics",
            DiagnosticArray,
            queue_size=10
        )

        self.p = psutil.Process()


    def update(self):
        status_cpu = DiagnosticStatus()
        status_cpu.name = "CPU"
        status_mem = DiagnosticStatus()
        status_mem.name = "Memory"
        status_disk = DiagnosticStatus()
        status_disk.name = "Disk"

        header = Header()
        header.stamp = rospy.Time.now()

        if self.sep_stats:
            cpu_temp = Float32Stamped()
            cpu_temp.header = header
            cpu = Float32Stamped()
            cpu.header = header
            disk = Float32Stamped()
            disk.header = header
            mem = Float32Stamped()
            mem.header = header
            swap = Float32Stamped()
            swap.header = header

        # one shot supposedly is faster since it uses cached values
        with self.p.oneshot():
            # cpu temp
            temps = psutil.sensors_temperatures()
            cpu_coretemp = None
            if 'coretemp' in temps:
                cpu_coretemp = mean(list(map(lambda x:x.current, temps['coretemp'])))
                status_cpu.values.append(KeyValue("coretemp", str(cpu_coretemp)))

            # cpu usage
            cpu_usage = mean(psutil.cpu_percent(percpu=True))
            status_cpu.values.append(KeyValue("usage", str(cpu_usage)))

            # disk
            disk_usage = psutil.disk_usage('/').percent
            status_disk.values.append(KeyValue("usage", str(disk_usage)))

            # memory
            mem_usage_virtual = psutil.virtual_memory().percent
            mem_usage_swap = psutil.swap_memory().percent
            status_mem.values.append(KeyValue("usage_virtual", str(mem_usage_virtual)))
            status_mem.values.append(KeyValue("usage_swap", str(mem_usage_swap)))

            if self.sep_stats:
                if cpu_coretemp is not None:
                    cpu_temp.data = cpu_coretemp
                cpu.data = cpu_usage
                disk.data = disk_usage
                mem.data = mem_usage_virtual
                swap.data = mem_usage_swap

                # publish
                self.cpu_temp_pub.publish(cpu_temp)
                self.cpu_pub.publish(cpu)
                self.disk_pub.publish(disk)
                self.mem_pub.publish(mem)
                self.swap_pub.publish(swap)

        # all stats
        msg = DiagnosticArray()
        msg.status = [status_cpu, status_mem, status_disk]
        msg.header = header

        self.stat_pub.publish(msg)