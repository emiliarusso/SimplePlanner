#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

struct Node {
  int idx;          // index in grid (y*W + x)
  float f, g, h;    // costs
  bool operator<(const Node& other) const { return f > other.f; } // min-heap
};

class SimplePlanner
{
public:
  SimplePlanner(ros::NodeHandle& nh): nh_(nh)
  {
    // Params
    nh_.param("alpha_clearance", alpha_clearance_, 0.0);      // peso distanza ostacoli
    nh_.param("obstacle_threshold", obstacle_thresh_, 50);    // soglia OccupancyGrid (0..100)

    map_sub_  = nh_.subscribe("/map", 1, &SimplePlanner::mapCb, this);
    init_sub_ = nh_.subscribe("/initialpose", 1, &SimplePlanner::initialPoseCb, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SimplePlanner::goalCb, this);

    path_pub_ = nh_.advertise<nav_msgs::Path>("/simple_planner/path", 1, true);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/simple_planner/markers", 1, true);

    // Param smoothing 
    nh_.param("smoothing_iters", smoothing_iters_, 2);   // 0 = no smoothing
    nh_.param("marker_width", marker_width_, 0.06);      // line width in meters

    frontier_pub_ = nh_.advertise<visualization_msgs::Marker>("/simple_planner/frontier", 1, true);
    visited_pub_  = nh_.advertise<visualization_msgs::Marker>("/simple_planner/visited", 1, true);
    costmap_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>("/simple_planner/cost_debug", 1, true);

    nh_.param("visualize", visualize_, true);
    nh_.param("viz_stride", viz_stride_, 200);
    nh_.param("world_frame", world_frame_, world_frame_);
    nh_.param("base_frame",  base_frame_,  base_frame_);
    nh_.param("use_tf_start", use_tf_start_, use_tf_start_);

    if(use_tf_start_){
      tf_timer_ = nh_.createTimer(ros::Duration(0.2), &SimplePlanner::updateStartFromTF, this);
      ROS_INFO("TF start allowed (using map->base_link).");
    } else {
      ROS_INFO("TF start DISABLED (using /initialpose from RViz).");
    }

    // Update start pose from TF at 5 Hz
    tf_timer_ = nh_.createTimer(ros::Duration(0.2), &SimplePlanner::updateStartFromTF, this);

    have_map_ = have_start_ = have_goal_ = false;
  }

private:
  // --- ROS ---
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_, init_sub_, goal_sub_;
  ros::Subscriber goal2_sub_ = nh_.subscribe("/move_base/goal", 1,
    &SimplePlanner::goalMoveBaseCb, this);
  ros::Publisher  path_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher frontier_pub_;   // cells in open list
  ros::Publisher visited_pub_;    // closed cells
  ros::Publisher costmap_pub_;    // cost grid
  bool visualize_ = true;
  int viz_stride_ = 200; // publishes every N expansions (to avoid saturation)

  // --- Map data ---
  nav_msgs::OccupancyGrid map_;
  double res_ = 0.05;
  double origin_x_ = 0.0, origin_y_ = 0.0;
  unsigned int W_ = 0, H_ = 0;

  // TF start
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::string world_frame_ = "map";
  std::string base_frame_  = "base_link";
  bool use_tf_start_ = true;              // enable TF for start
  ros::Timer tf_timer_;

  // cost_: 255 obstacle/unknown, 0 free
  std::vector<unsigned char> cost_;
  // dist_: distance in meters from free cell to nearest obstacle (>=0), +inf if no obstacle (rare)
  std::vector<float> dist_;

  // Start/Goal in world
  geometry_msgs::Point start_w_, goal_w_;
  double start_yaw_ = 0.0, goal_yaw_ = 0.0;
  bool have_map_, have_start_, have_goal_;

  // Params
  double alpha_clearance_;
  int obstacle_thresh_;

  int smoothing_iters_ = 2;
  double marker_width_ = 0.06;

  // --- Utils ---
  static double yawFromQuat(const geometry_msgs::Quaternion& q){
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);
    double r, p, y; tf2::Matrix3x3(qq).getRPY(r,p,y);
    return y;
  }

  inline bool worldToGrid(double wx,double wy,int& gx,int& gy) const {
    gx = (int)std::floor((wx - origin_x_) / res_);
    gy = (int)std::floor((wy - origin_y_) / res_);
    return (gx>=0 && gy>=0 && gx<(int)W_ && gy<(int)H_);
  }
  inline void gridToWorld(int gx,int gy,double& wx,double& wy) const {
    wx = origin_x_ + (gx + 0.5) * res_;
    wy = origin_y_ + (gy + 0.5) * res_;
  }
  inline int toIdx(int x,int y) const { return y*(int)W_ + x; }
  inline void fromIdx(int idx,int& x,int& y) const { y = idx / (int)W_; x = idx - y*(int)W_; }

  // --- Callbacks ---
  void mapCb(const nav_msgs::OccupancyGrid& msg){
    map_ = msg;
    res_ = msg.info.resolution;
    origin_x_ = msg.info.origin.position.x;
    origin_y_ = msg.info.origin.position.y;
    W_ = msg.info.width;
    H_ = msg.info.height;

    const size_t N = (size_t)W_ * (size_t)H_;
    cost_.assign(N, 255);
    for(size_t i=0;i<N;++i){
      int8_t v = map_.data[i];
      // -1 unknown -> obstacle; >= obstacle_thresh_ -> obstacle
      cost_[i] = (v < 0 || v >= obstacle_thresh_) ? 255 : 0;
    }

    computeObstacleDistances(); // fills dist_ in meters
    publishCostDebug();
    have_map_ = true;

    tryPlan(); // if start/goal already known, try immediately
  }

  void initialPoseCb(const geometry_msgs::PoseWithCovarianceStamped& m){
    start_w_ = m.pose.pose.position;
    start_yaw_ = yawFromQuat(m.pose.pose.orientation);
    have_start_ = true;
    tryPlan();
  }
  void goalCb(const geometry_msgs::PoseStamped& m){
    goal_w_ = m.pose.position;
    goal_yaw_ = yawFromQuat(m.pose.orientation);
    have_goal_ = true;
    tryPlan();
  }

  void updateStartFromTF(const ros::TimerEvent&){
    if(!use_tf_start_) return;
    try{
      auto tf = tf_buffer_.lookupTransform(world_frame_, base_frame_, ros::Time(0));
      start_w_.x = tf.transform.translation.x;
      start_w_.y = tf.transform.translation.y;
      start_yaw_ = yawFromQuat(tf.transform.rotation);
      have_start_ = true;
      // Plan when goal or map arrives.
    } catch(const tf2::TransformException& e){
      ROS_WARN_THROTTLE(2.0, "TF %s->%s not available: %s",
                        world_frame_.c_str(), base_frame_.c_str(), e.what());
    }
  }

  void goalMoveBaseCb(const move_base_msgs::MoveBaseActionGoal& msg){
    goal_w_  = msg.goal.target_pose.pose.position;
    goal_yaw_ = yawFromQuat(msg.goal.target_pose.pose.orientation);
    have_goal_ = true;
    tryPlan();
  }

  // --- Distance transform (BFS from obstacles) ---
  void computeObstacleDistances(){
    const size_t N = (size_t)W_ * (size_t)H_;
    dist_.assign(N, std::numeric_limits<float>::infinity());

    std::queue<int> q;
    // Initialize with all obstacle cells (distance 0)
    for(size_t i=0;i<N;++i){
      if(cost_[i] == 255){
        dist_[i] = 0.0f;
        q.push((int)i);
      }
    }
    // 4-neighbors BFS for grid steps
    const int dx[4] = {1,-1,0,0};
    const int dy[4] = {0,0,1,-1};
    while(!q.empty()){
      int cur = q.front(); q.pop();
      int cx, cy; fromIdx(cur, cx, cy);
      for(int k=0;k<4;++k){
        int nx = cx + dx[k], ny = cy + dy[k];
        if(nx<0||ny<0||nx>=(int)W_||ny>=(int)H_) continue;
        int ni = toIdx(nx,ny);
        if(dist_[ni] == std::numeric_limits<float>::infinity()){
          dist_[ni] = dist_[cur] + 1.0f; // grid steps
          q.push(ni);
        }
      }
    }
    // Convert steps to meters
    for(size_t i=0;i<N;++i){
      if(std::isfinite(dist_[i])) dist_[i] *= (float)res_;
      else                         dist_[i] = 1e6f; // very far
    }
  }

  // --- CHaikin Smoothing function ----
  void smoothChaikin(std::vector<geometry_msgs::PoseStamped>& pts, int iters){
    if(iters <= 0 || pts.size() < 3) return;
    for(int it=0; it<iters; ++it){
      std::vector<geometry_msgs::PoseStamped> out;
      out.reserve(pts.size()*2);
      out.push_back(pts.front()); // keep endpoints
      for(size_t i=0;i+1<pts.size();++i){
        const auto& P = pts[i].pose.position;
        const auto& Q = pts[i+1].pose.position;
        geometry_msgs::PoseStamped A = pts[i];
        geometry_msgs::PoseStamped B = pts[i];
        A.pose.position.x = 0.75*P.x + 0.25*Q.x;
        A.pose.position.y = 0.75*P.y + 0.25*Q.y;
        B.pose.position.x = 0.25*P.x + 0.75*Q.x;
        B.pose.position.y = 0.25*P.y + 0.75*Q.y;
        out.push_back(A);
        out.push_back(B);
      }
      out.push_back(pts.back());
      pts.swap(out);
    }
  }

  visualization_msgs::Marker makeLineStrip(const nav_msgs::Path& path, int id){
    visualization_msgs::Marker m;
    m.header = path.header;
    m.ns = "simple_planner";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = marker_width_; // line thickness
    // blue color
    m.color.r = 0.117; m.color.g = 0.466; m.color.b = 0.741; m.color.a = 1.0;
    m.pose.orientation.w = 1.0;
    m.points.reserve(path.poses.size());
    for(const auto& p : path.poses){
      geometry_msgs::Point pt;
      pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = 0.02;
      m.points.push_back(pt);
    }
    return m;
  }

  visualization_msgs::Marker makeSphere(const geometry_msgs::Point& pw, const std_msgs::Header& h, int id, bool is_start){
    visualization_msgs::Marker m;
    m.header = h;
    m.ns = "simple_planner";
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position = pw;
    m.pose.position.z = 0.05;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = std::max(0.25, marker_width_*4.0);
    if(is_start){
      // blue
      m.color.r = 0.117; m.color.g = 0.466; m.color.b = 0.741; m.color.a = 1.0;
    }else{
      // green
      m.color.r = 0.204; m.color.g = 0.596; m.color.b = 0.157; m.color.a = 1.0;
    }
    return m;
  }

  visualization_msgs::Marker makeCubeList(const std::vector<int>& cells,
                                        const std_msgs::Header& h,
                                        int id, float r,float g,float b,float a){
    visualization_msgs::Marker m;
    m.header = h;
    m.ns = "simple_planner_debug";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = std::max(0.8*res_, 0.02); // almost as big as the cell
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.pose.orientation.w = 1.0;
    m.points.reserve(cells.size());
    for(int idx: cells){
      int x,y; fromIdx(idx,x,y);
      double wx,wy; gridToWorld(x,y,wx,wy);
      geometry_msgs::Point p; p.x=wx; p.y=wy; p.z=0.01;
      m.points.push_back(p);
    }
    return m;
  }

  void publishMarkers(const nav_msgs::Path& path){
    if(path.poses.empty()) return;
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(makeLineStrip(path, 0));
    arr.markers.push_back(makeSphere(path.poses.front().pose.position, path.header, 1, true));  // start
    arr.markers.push_back(makeSphere(path.poses.back().pose.position,  path.header, 2, false)); // goal
    marker_pub_.publish(arr);
  }

  void publishCostDebug(){
    if(costmap_pub_.getNumSubscribers()==0) return;
    nav_msgs::OccupancyGrid og = map_;
    og.data.resize(W_*H_);
    for(size_t i=0;i<og.data.size();++i){
      if(cost_[i]==255) { og.data[i]=100; continue; }
      // map distance to [0,100] (near walls = higher)
      float d = std::min(dist_[i], 1.0f); // clamp to 1m
      int v = (int)std::round((1.0f - d/1.0f)*80.0f); // 0..80
      og.data[i] = v;
    }
    costmap_pub_.publish(og);
  }

  // --- Planner ---
  void tryPlan(){
    if(!(have_map_ && have_start_ && have_goal_)) return;

    int sx, sy, gx, gy;
    if(!worldToGrid(start_w_.x, start_w_.y, sx, sy)){
        ROS_WARN_THROTTLE(1.0, "Start out of map");
        return;
    }
    if(!worldToGrid(goal_w_.x, goal_w_.y, gx, gy)){
        ROS_WARN_THROTTLE(1.0, "Goal out of map");
        return;
    }

    int sidx = toIdx(sx, sy);
    int gidx = toIdx(gx, gy);
    if(cost_[sidx] == 255 || cost_[gidx] == 255){
        ROS_WARN_THROTTLE(1.0, "Start/Goal in non-traversable cell");
        return;
    }

    nav_msgs::Path path;
    if(!planAStar(sidx, gidx, path)){
        ROS_WARN_THROTTLE(1.0, "A* did not find a path");
        return;
    }

    // Apply optional smoothing
    if(smoothing_iters_ > 0){
        smoothChaikin(path.poses, smoothing_iters_);
    }

    // Update path header
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    // Publish path
    path_pub_.publish(path);
    ROS_INFO("Path with %zu poses published to /simple_planner/path", path.poses.size());

    // Publish marker: thick line + start/goal
    publishMarkers(path);
  }

  bool planAStar(int start_idx, int goal_idx, nav_msgs::Path& out_path){
    const size_t N = (size_t)W_ * (size_t)H_;
    std::vector<float> gScore(N, std::numeric_limits<float>::infinity());
    std::vector<int> cameFrom(N, -1);
    std::vector<char> closed(N, 0);

    // --- vectors for visual debugging ---
    std::vector<int> frontier_cells, visited_cells;
    size_t iters = 0;

    auto heuristic = [&](int a, int b)->float{
      int ax, ay, bx, by; fromIdx(a,ax,ay); fromIdx(b,bx,by);
      float dx = (float)(ax - bx), dy = (float)(ay - by);
      return std::sqrt(dx*dx + dy*dy); // in celle
    };

    std::priority_queue<Node> open;
    gScore[start_idx] = 0.f;
    open.push(Node{start_idx, heuristic(start_idx, goal_idx), 0.f, heuristic(start_idx, goal_idx)});

    const int dx8[8] = {1,-1,0,0, 1,1,-1,-1};
    const int dy8[8] = {0,0,1,-1, 1,-1,1,-1};
    const float move_cost[8] = {1,1,1,1, std::sqrt(2.f),std::sqrt(2.f),std::sqrt(2.f),std::sqrt(2.f)};

    while(!open.empty()){
      Node cur = open.top(); open.pop();
      if(closed[cur.idx]) continue;
      closed[cur.idx] = 1;

      // ---- DEBUG: add visited cell
      visited_cells.push_back(cur.idx);

      if(cur.idx == goal_idx){
        reconstructPath(cameFrom, goal_idx, out_path);

        // ---- DEBUG: publish one last time visited/frontier
        if(visualize_){
          std_msgs::Header hdr; hdr.stamp = ros::Time::now(); hdr.frame_id = "map";
          visited_pub_.publish( makeCubeList(visited_cells, hdr, 10, 0.9, 0.9, 0.0, 0.6) );
          frontier_pub_.publish( makeCubeList(frontier_cells, hdr, 11, 0.2, 0.6, 1.0, 0.6) );
        }
        return true;
      }

      int cx, cy; fromIdx(cur.idx, cx, cy);
      for(int k=0;k<8;++k){
        int nx = cx + dx8[k], ny = cy + dy8[k];
        if(nx<0||ny<0||nx>=(int)W_||ny>=(int)H_) continue;
        int ni = toIdx(nx,ny);
        if(cost_[ni]==255) continue;       // obstacle
        if(closed[ni]) continue;

        // base cost + wall proximity penalty
        float base = move_cost[k];
        float clearance_pen = (float)alpha_clearance_ * (1.0f / (1e-3f + dist_[ni]));
        float tentative_g = gScore[cur.idx] + base + clearance_pen;

        if(tentative_g < gScore[ni]){
          gScore[ni] = tentative_g;
          cameFrom[ni] = cur.idx;
          float h = heuristic(ni, goal_idx);
          open.push(Node{ni, tentative_g + h, tentative_g, h});

          // ---- DEBUG: add cell to frontier
          frontier_cells.push_back(ni);
        }
      }

      // ---- DEBUG: PERIODIC PUBLISHING EVERY viz_stride_ ITERATIONS
      if(visualize_ && (++iters % (size_t)viz_stride_ == 0)){
        std_msgs::Header hdr; hdr.stamp = ros::Time::now(); hdr.frame_id = "map";
        visited_pub_.publish( makeCubeList(visited_cells, hdr, 10, 0.9, 0.9, 0.0, 0.6) ); // yellow
        frontier_pub_.publish( makeCubeList(frontier_cells, hdr, 11, 0.2, 0.6, 1.0, 0.6) ); // blue
        frontier_cells.clear(); // avoid republishing the same cells
      }
    }

    return false;
  }

  void reconstructPath(const std::vector<int>& cameFrom, int goal_idx, nav_msgs::Path& path){
    std::vector<int> rev;
    for(int cur = goal_idx; cur!=-1; cur = cameFrom[cur]) rev.push_back(cur);
    std::reverse(rev.begin(), rev.end());

    path.poses.clear();
    path.poses.reserve(rev.size());
    for(int idx : rev){
      int x,y; fromIdx(idx,x,y);
      double wx, wy; gridToWorld(x,y,wx,wy);
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0; // no specific orientation for waypoints
      path.poses.push_back(ps);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_planner_node");
  ros::NodeHandle nh("~"); // private ns for parameters
  SimplePlanner planner(nh);
  ros::spin();
  return 0;
}