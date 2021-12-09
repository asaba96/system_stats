# README #

System statistics (CPU/RAM/DISK) & node process monitor

### What is this repository for? ###

Provides a ROS interface around psutil to grab memory, cpu, and other stats about the running nodes on a system and overal stats for the system. 

For node monitoring, this tool also keeps track of which nodes have died and how many times they have been restarted, if it all. This is useful for monitoring the health of a given system and seeing if a critical node is up. 

### How do I get set up? ###

Tested with ROS Melodic 18.04

Only dendency is psutil which can be installed with `pip install psutil`

To launch, run `roslaunch system_stats system_monitor.launch`

Launch file provides the following parmaters:

*  pub_nodes: default true, whether or not to publish per node stats
*  pub_dead_nodes: default true, whether or not to publish a separate topic for dead nodes
*  separate_stats: default true, whether or not to publish system stats on separate topics or just one diagnostics topic

### Contribution guidelines ###

Future additions:

*  GPU monitoring
*  Additional temperature stats
*  More memory and CPU usage breakdowns overall
*  Any additional efficiency improvements

### Who do I talk to? ###

Andrew Saba (asaba@andrew.cmu.edu)