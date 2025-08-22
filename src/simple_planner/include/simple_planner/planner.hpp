#pragma once
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <cstdint>
#include "simple_planner/grid_utils.hpp"

// cell representation
struct GridIndex { int r{0}, c{0}; };

// output of the A* planner
struct PlanResult { bool ok{false}; std::vector<GridIndex> cells; double cost{0.0}; };

// A* planner class operating on 2D grid 
class AStarPlanner {
public:
  AStarPlanner(const GridGeom& g, // grid geometry
               const std::vector<uint8_t>& occ, // occupancy grid 0/1
               const std::vector<float>& clearance_cost, 
               double alpha) // weight for clearance cost
  : geom_(g), occ_(occ), cc_(clearance_cost), alpha_(alpha) {}

  // runs A* from start to goal
  PlanResult plan(GridIndex start, GridIndex goal); // returns path cells, success flag, and cost

private:
  GridGeom geom_;                        // grid metadata (dimensions, resolution)
  const std::vector<uint8_t>& occ_;      // occupancy map (obstacles)
  const std::vector<float>& cc_;         // clearance cost per cell
  double alpha_;                         // weight for clearance in the cost function

  // Checks if a grid cell (r,c) is free (inside bounds and not occupied).
  inline bool freeCell(int r,int c) const {
    return inBounds(r,c,geom_.rows,geom_.cols) 
           && !occ_[idx(r,c,geom_.cols)];
  }
};