# inspiration taken from
#   https://github.com/ros-visualization/rqt_top/blob/master/src/rqt_top/node_info.py
# modified by Andrew Saba

import rosnode
import rospy
import psutil
import threading
import traceback

from socket import error as SocketError

from system_stats.msg import NodeState, NodeStateArray

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy


class Node(object):
    # wrapper around psutil process
    # with info about life, # of respawns, etc.

    # memory stats added based on this blog post
    # https://grodola.blogspot.com/2016/02/psutil-4-real-process-memory-and-environ.html

    def __init__(self, name, process):
        self.name = name
        self.process = process

        self.is_alive = True
        # num of times has been respawned since first starting
        self.num_respawns = 0

    def update(self):
        msg = NodeState()
        msg.name = self.name = self.name
        msg.alive = self.is_alive
        msg.restart_count = self.num_respawns

        if self.is_alive:
            try:
                # grab with one shot to optimize calls
                with self.process.oneshot():
                    msg.memory = self.process.memory_percent()
                    msg.cpu = self.process.cpu_percent()

                    cpu_times = self.process.cpu_times()
                    mem_info = self.process.memory_full_info()

                    msg.user = cpu_times.user
                    msg.system = cpu_times.system
                    msg.children_user = cpu_times.children_user
                    msg.children_system = cpu_times.children_system
                    msg.iowait = cpu_times.iowait

                    msg.rss = mem_info.rss
                    msg.pss = mem_info.pss
                    msg.uss = mem_info.uss
                    msg.swap = mem_info.swap
            except psutil.NoSuchProcess:
                rospy.logwarn("NodeInfo: node {} has died".format(self.name))
                self.is_alive = False
                msg.alive = False

        return msg


class NodeInfo(object):

    def __init__(self, pub_dead):
        self._pub_dead = pub_dead
        self.nodes = dict()
        self.p = psutil.Process()

        self._node_pub = rospy.Publisher(
            "system/node_stats",
            NodeStateArray,
            queue_size=10
        )

        if self._pub_dead:
            self._dead_node_pub = rospy.Publisher(
                "system/dead_nodes",
                NodeStateArray,
                queue_size=10
            )

        # thread safe
        self._lock = threading.RLock()

        # timer to check for contacting master and checking for new nodes
        self._timer = rospy.Timer(rospy.Duration(1.0/4.0), self._update_nodes)

    def _update_nodes(self, event):
        # check master to see which nodes are up
        running_nodes = rosnode.get_node_names()

        with self._lock:
            # check for dead & respawned nodes
            for name, node in self.nodes.items():
                if name not in running_nodes:
                    # dead node
                    node.is_alive = False
                elif not node.is_alive:
                    # update respawned nodes
                    p = self.get_node_info(name)
                    if p is not None:
                        node.is_alive = True
                        node.num_respawns = node.num_respawns + 1
                        node.process = p

            # insert new nodes
            for node_ in running_nodes:
                if node_ not in self.nodes:
                    p = self.get_node_info(node_)

                    # see get_node_info for why we eat error
                    if p is not None:
                        n = Node(node_, p)
                        self.nodes[node_] = n

    def get_node_info(self, node_name, skip_cache=False):
        # call node to get PIDs
        node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=skip_cache)
        try:
            code, msg, pid = ServerProxy(node_api[2]).getPid('/NODEINFO')
            try:
                p = psutil.Process(pid)
                return p
            except Exception as err:
                rospy.logerr("NodeInfo: cannot retrieve process for node {} {}".format(node_name, str(err)))
                return None
        except SocketError:
            # node has likely died but is still registered with ROS
            # ignore error for now, future look into maybe cleaning up ros automatically
            if not skip_cache:
                # not sure how good it is to have this here, comment out for now
                # rosnode.rosnode_cleanup()
                return self.get_node_info(node_name, skip_cache=True)
            else:
                return None

    def update(self):
        # main update loop
        msg = NodeStateArray()
        msg.header.stamp = rospy.Time.now()

        # message for dead nodes only
        dead = NodeStateArray()
        dead.header.stamp = rospy.Time.now()

        with self._lock:
            # update stats for each node
            for name, node in self.nodes.items():
                try:
                    m = node.update()
                except psutil.NoSuchProcess:
                    # shouldnt happen
                    rospy.logwarn("NodeInfo: node {} has died".format(name))
                    node.is_alive = False
                except Exception as e:
                    # ignore node this time since something unexpected happened
                    rospy.logerr("NodeInfo: error updating node {}".format(name))
                    rospy.logerr("NodeInfo:\n {}".format(traceback.format_exc(e)))
                    node.is_alive = False
                else:
                    if not node.is_alive:
                        dead.nodes.append(m)
                    msg.nodes.append(m)

        # publish
        self._node_pub.publish(msg)

        if self._pub_dead:
            self._dead_node_pub.publish(dead)

    def kill_node(self, node_name):
        success, fail = rosnode.kill_nodes([node_name])
        return node_name in success
