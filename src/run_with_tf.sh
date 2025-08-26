#!/usr/bin/env bash
set -e
source ~/SimplePlanner/devel/setup.bash

# T1 - main
roscore & 
sleep 1

# T2 - map
rosrun map_server map_server $(rospack find simple_planner)/maps/map.yaml &
sleep 0.5

# T3 - start from simulated TF 
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 &
sleep 0.2

# T4 - planner (uses TF for start)
rosrun simple_planner simple_planner_node _use_tf_start:=true &
sleep 0.2

# T5 - RViz config
rviz -d $(rospack find simple_planner)/rviz/simple_planner_topdown.rviz