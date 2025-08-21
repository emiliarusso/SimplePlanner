#pragma once
#include <vector>
#include <cstdint>
#include "simple_planner/grid_utils.hpp"

// struct that stores three main layers of data
// occupancy, distance, and cost
struct Costmap {
  GridGeom geom;              // grid geometry
  std::vector<uint8_t> occ;   // 0=free, 1=obstacle
  std::vector<float> dist;    // m to closest obstacle
  std::vector<float> cost;    // clearance cost = w/(eps+d), higher near obstacles
};

void computeObstacleDistances(Costmap& cm); // euclidean distance from cell to closest obstacle
void computeClearanceCost(Costmap& cm, double w_clearance=1.0, double eps=1e-3); // cost from distance knowledge