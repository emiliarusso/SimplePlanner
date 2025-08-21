#pragma once
#include <cmath>

struct GridGeom {
  int rows{0}, cols{0}; // grid size (rows x cols)
  double resolution{0.05};  // meters per cell
  double origin_x{0.0}, origin_y{0.0}; // world of cell (0,0)
};

// convert world coordinates to grid coordinates
// wx,wy: world coordinates
// r,c: grid coordinates (indices)
inline bool worldToGrid(double wx, double wy, int& r, int& c, const GridGeom& g){
  int gc = static_cast<int>(std::floor((wx - g.origin_x) / g.resolution));
  int gr = static_cast<int>(std::floor((wy - g.origin_y) / g.resolution));
  if(gr < 0 || gr >= g.rows || gc < 0 || gc >= g.cols) return false;
  r = gr; c = gc; return true;
}

// convert grid indices in world coordinates 
// used to visualize correctly in rviz 
inline void gridToWorld(int r, int c, double& wx, double& wy, const GridGeom& g){
  wx = g.origin_x + (c + 0.5) * g.resolution;
  wy = g.origin_y + (r + 0.5) * g.resolution;
}

// Convert row/col indices -> 1D array index
inline int idx(int r, int c, int cols){ return r*cols + c; }

// bounds helper
inline bool inBounds(int r,int c,int R,int C){ return r>=0 && r<R && c>=0 && c<C; }