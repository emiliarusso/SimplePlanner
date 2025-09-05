#include "simple_planner/costmap.hpp"
#include <queue>
#include <limits>

// fills the distance map with euclidean distances from each cell to the nearest obstacle
void computeObstacleDistances(Costmap& cm){
  const int R = cm.geom.rows, C = cm.geom.cols;
  cm.dist.assign(R*C, std::numeric_limits<float>::infinity()); // initialize all distances to infinity
  std::queue<int> q; // queue for BFS

  // initialize queue with obstacle cells (obstacle distance = 0)
  for(int r=0;r<R;++r) for(int c=0;c<C;++c){
    if(cm.occ[idx(r,c,C)]){ cm.dist[idx(r,c,C)] = 0.f; q.push(idx(r,c,C)); }
  }

  // 4-connected neighbors 
  const int dr[4]={-1,1,0,0}, dc[4]={0,0,-1,1};

  // BFS expansion
  while(!q.empty()){
    int id=q.front(); q.pop();
    int r=id/C, c=id% C;
    for(int k=0;k<4;++k){  // for each neighbor
      int nr=r+dr[k], nc=c+dc[k];
      if(!inBounds(nr,nc,R,C)) continue; // skip out of bounds
      int nid=idx(nr,nc,C);
      float cand = cm.dist[id] + cm.geom.resolution; // candidate distance = parent distance + resolution
      if(cand < cm.dist[nid]){ cm.dist[nid]=cand; q.push(nid); }
    }
  }
}

// computes the clearance cost based on the distance map
void computeClearanceCost(Costmap& cm, double w_clearance, double eps){
  const int N = cm.geom.rows * cm.geom.cols;
  cm.cost.resize(N);
  for(int i=0;i<N;++i){
    float d = cm.dist[i];
    cm.cost[i] = w_clearance / (eps + d); // near obstacle -> higher cost
  }
}