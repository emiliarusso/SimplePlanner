#!/usr/bin/env bash
set -e
source ~/SimplePlanner/devel/setup.bash
roscore & sleep 1
rosrun map_server map_server $(rospack find simple_planner)/maps/map.yaml &
rviz -d $(rospack find simple_planner)/rviz/simple_planner_topdown.rviz &
rosrun simple_planner simple_planner_node _use_tf_start:=false
