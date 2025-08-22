#include "simple_planner/planner.hpp"

// Node represents one cell in the A* search
struct Node { 
  int r, c;      // grid coordinates (row, col)
  double g, f;   // g = cost so far, f = g + heuristic
  int pr, pc;    // parent cell (row, col) to reconstruct path
};

// Comparator for priority queue 
struct Cmp { 
  bool operator()(const Node& a, const Node& b) const { 
    return a.f > b.f; 
  } 
};

PlanResult AStarPlanner::plan(GridIndex s, GridIndex g){
  PlanResult out; 
  out.ok = false;

  // If start or goal is not free, return failure
  if(!freeCell(s.r,s.c) || !freeCell(g.r,g.c)) 
    return out;

  // Grid dimensions
  const int R = geom_.rows, 
            C = geom_.cols, 
            N = R * C;   // total number of cells

  // gscore[id] = best cost found so far to reach cell id
  std::vector<double> gscore(N, std::numeric_limits<double>::infinity());

  // parent[id] = previous cell index used to reconstruct path
  std::vector<int> parent(N, -1);

  // Open set = frontier of nodes to explore, ordered by lowest f
  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  // Heuristic function (Euclidean distance to goal)
  auto H = [&](int r,int c){ 
    double dr = r - g.r, dc = c - g.c; 
    return std::sqrt(dr*dr + dc*dc); 
  };

  // Push function: adds node to open list if newg is better
  auto push = [&](int r,int c,int pr,int pc,double newg){
    int id = idx(r,c,C);                 // map (r,c) → linear index
    if(newg >= gscore[id]) return;       // ignore if not an improvement
    gscore[id] = newg;                   // update best cost
    double f = newg + H(r,c);            // f = g + h
    open.push(Node{r,c,newg,f,pr,pc});   // add to frontier
    parent[id] = (pr < 0 ? -1 : idx(pr,pc,C)); // store parent link
  };

  // Initialize with the start cell
  push(s.r, s.c, -1, -1, 0.0);

  // 8-connected neighbors 
  const int dr[8]   = {-1,-1,-1, 0, 0, 1, 1, 1};
  const int dc[8]   = {-1, 0, 1,-1, 1,-1, 0, 1};
  const double step[8] = {
    std::sqrt(2), 1, std::sqrt(2), 1, 1, std::sqrt(2), 1, std::sqrt(2)
  }; // movement cost per direction

  while(!open.empty()){
    Node cur = open.top(); open.pop();

    // Check if we reached the goal
    if(cur.r == g.r && cur.c == g.c){
      std::vector<GridIndex> rev;
      int id = idx(cur.r, cur.c, C);
      // Backtrack using parent[] to reconstruct path
      while(id != -1){ 
        int rr = id / C, cc = id % C; 
        rev.push_back({rr,cc}); 
        id = parent[id]; 
      }
      // Reverse the reconstructed path
      out.cells.assign(rev.rbegin(), rev.rend());
      out.cost = cur.g; 
      out.ok = true; 
      return out; // success
    }

    // Expand neighbors
    for(int k=0;k<8;++k){
      int nr = cur.r + dr[k], 
          nc = cur.c + dc[k];
      if(!freeCell(nr,nc)) continue;   // skip obstacles/out of bounds
      int nid = idx(nr,nc,C);

      // Cost adjustment
      double add = 1.0 + alpha_ * cc_[nid];
      double ng = cur.g + step[k] * add;  
      push(nr, nc, cur.r, cur.c, ng);    
    }
  }

  // If frontier is exhausted, no path exists
  return out;
}