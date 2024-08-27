You can perform profiling on roslaunch or rosrun. See tutorial: http://wiki.ros.org/roslaunch/Tutorials/Profiling%20roslaunch%20nodes

Tested with 
rosrun --prefix 'valgrind --tool=callgrind' hear_mc_example mc_graph_based_node
and then running GUI kcachegrind