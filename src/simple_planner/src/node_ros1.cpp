#include "simple_planner/costmap.hpp"
#include "simple_planner/planner.hpp"
#include "simple_planner/grid_utils.hpp"
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

class SimplePlanner
{
public:
  SimplePlanner(ros::NodeHandle& nh): nh_(nh)
  {
    // Params
    nh_.param("alpha_clearance", alpha_clearance_, 0.0);      // weight for clearance cost
    nh_.param("obstacle_threshold", obstacle_thresh_, 50);    // threshold for OccupancyGrid (0..100)

    map_sub_  = nh_.subscribe("/map", 1, &SimplePlanner::mapCb, this);
    init_sub_ = nh_.subscribe("/initialpose", 1, &SimplePlanner::initialPoseCb, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SimplePlanner::goalCb, this);
    goal2_sub_ = nh_.subscribe("/move_base/goal", 1, &SimplePlanner::goalMoveBaseCb, this);

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

    have_map_ = have_start_ = have_goal_ = false;
  }

private:
  // --- ROS ---
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_, init_sub_, goal_sub_;
  ros::Subscriber goal2_sub_;
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

  Costmap cm_;          // holds geom, occ, dist, cost
  std::unique_ptr<AStarPlanner> astar_;  // the planner that uses geom + clearance cost

  // TF start
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::string world_frame_ = "map";
  std::string base_frame_  = "base_link";
  bool use_tf_start_ = true;              // enable TF for start
  ros::Timer tf_timer_;

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

  // Build a GridGeom from current map metadata
  inline GridGeom g() const {
    GridGeom gg;
    gg.rows = static_cast<int>(H_);
    gg.cols = static_cast<int>(W_);
    gg.resolution = res_;
    gg.origin_x = origin_x_;
    gg.origin_y = origin_y_;
    return gg;
  }

  // Wrapper: world -> grid 
  inline bool worldToGrid(double wx,double wy,int& gx,int& gy) const {
    int r, c;
    if(!::worldToGrid(wx, wy, r, c, g())) return false;  // shared util (row, col)
    gx = c;  // x = col
    gy = r;  // y = row
    return true;
  }

  // Wrapper: grid -> world 
  inline void gridToWorld(int gx,int gy,double& wx,double& wy) const {
    ::gridToWorld(gy, gx, wx, wy, g());  // note (row=gy, col=gx)
  }

  // Wrapper: 2D -> 1D
  inline int toIdx(int x,int y) const {           // x=col, y=row
    return ::idx(y, x, static_cast<int>(W_));     // shared util expects (row, col, cols)
  }

  // Wrapper: 1D -> 2D
  inline void fromIdx(int idx,int& x,int& y) const {
    y = idx / static_cast<int>(W_);               // row
    x = idx - y * static_cast<int>(W_);           // col
  }

  // --- Callbacks ---
  void mapCb(const nav_msgs::OccupancyGrid& msg){
    map_ = msg;

    // metadata
    res_      = msg.info.resolution;
    origin_x_ = msg.info.origin.position.x;
    origin_y_ = msg.info.origin.position.y;
    W_        = msg.info.width;
    H_        = msg.info.height;

    // geometry
    cm_.geom.rows       = static_cast<int>(msg.info.height);
    cm_.geom.cols       = static_cast<int>(msg.info.width);
    cm_.geom.resolution = msg.info.resolution;
    cm_.geom.origin_x   = msg.info.origin.position.x;
    cm_.geom.origin_y   = msg.info.origin.position.y;

    // occupancy (true = obstacle, false = free)
    const int R = cm_.geom.rows, C = cm_.geom.cols;
    cm_.occ.assign(R*C, false);
    for(int r=0; r<R; ++r){
      for(int c=0; c<C; ++c){
        int8_t v = msg.data[r*C + c]; // 0..100, -1 unknown
        bool obstacle = (v < 0) || (v >= obstacle_thresh_);
        cm_.occ[idx(r, c, C)] = obstacle;
      }
    }

    // distance field (meters) via library
    computeObstacleDistances(cm_);

    // clearance cost cc[i] = 1/(eps + dist[i])   
    computeClearanceCost(cm_, 1.0, 1e-3); 

    // configure planner with geometry + clearance and alpha
    astar_.reset(new AStarPlanner(cm_.geom, cm_.occ, cm_.cost, alpha_clearance_));
    astar_->setVizCallback(
      [this](const std::vector<int>& frontier, const std::vector<int>& visited){
        std_msgs::Header h; h.frame_id = world_frame_; h.stamp = ros::Time::now();
        auto mF = makeCubeList(frontier, h, 100, 1.0f, 0.6f, 0.0f, 0.5f);   // orange frontier
        auto mV = makeCubeList(visited,  h, 101, 0.121f,0.466f,0.705f,0.35f); // blue visited
        frontier_pub_.publish(mF);
        visited_pub_.publish(mV);
      },
      viz_stride_   // you already have this param
    );

    // publish debug costmap
    publishCostDebugFrom(cm_); 

    have_map_ = true;
    tryPlan();
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
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = std::max(0.8*res_, 0.02); 
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;

    if(cells.empty()){
      m.action = visualization_msgs::Marker::DELETE;
      return m; // nothing to draw, but clears any old cubes
    }

    m.action = visualization_msgs::Marker::ADD;
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

  void publishCostDebugFrom(const Costmap& cm){
    if(costmap_pub_.getNumSubscribers()==0) return;
    nav_msgs::OccupancyGrid og = map_;
    og.data.resize(cm.geom.rows * cm.geom.cols);

    // map distance in [0,1m] to [0,80] (near walls brighter)
    for(int r=0; r<cm.geom.rows; ++r){
      for(int c=0; c<cm.geom.cols; ++c){
        int id = idx(r,c,cm.geom.cols);
        if(cm.occ[id]) { og.data[id] = 100; continue; }
        float d = std::min(cm.dist[id], 1.0f);
        int v = static_cast<int>(std::round((1.0f - d/1.0f)*80.0f));
        og.data[id] = v;
      }
    }
    costmap_pub_.publish(og);
  }

  // --- Planner ---
  void tryPlan(){
    // Must have all inputs ready
    if(!(have_map_ && have_start_ && have_goal_)) return;
    if(W_ == 0 || H_ == 0){  // safety: empty map
      ROS_WARN_THROTTLE(1.0, "Map has zero size (W or H == 0).");
      return;
    }

    // World (m) -> Grid (cells)
    int sx, sy, gx, gy;
    if(!worldToGrid(start_w_.x, start_w_.y, sx, sy)){
      ROS_WARN_THROTTLE(1.0, "Start out of map bounds");
      return;
    }
    if(!worldToGrid(goal_w_.x, goal_w_.y, gx, gy)){
      ROS_WARN_THROTTLE(1.0, "Goal out of map bounds");
      return;
    }

    // Indices + traversability check
    const int sidx = toIdx(sx, sy);
    const int gidx = toIdx(gx, gy);
    if(cm_.occ[sidx] || cm_.occ[gidx]){  // true = obstacle/unknown
      ROS_WARN_THROTTLE(1.0, "Start/Goal is in a non-traversable cell (obstacle/unknown)");
      return;
    }

    // Plan
    if(!astar_){
      ROS_WARN_THROTTLE(1.0, "Planner not initialized yet.");
      return;
    }

    // Build start/goal GridIndex in (row, col)
    GridIndex S{sy, sx};
    GridIndex G{gy, gx};

    auto result = astar_->plan(S, G);
    if(!result.ok){
      ROS_WARN_THROTTLE(1.0, "A* did not find a path");
      return;
    }

    // Convert result.cells -> nav_msgs::Path
    nav_msgs::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp = ros::Time::now();
    path.poses.reserve(result.cells.size());
    for(const auto& gc : result.cells){
      double wx, wy;
      gridToWorld(/*gx=*/gc.c, /*gy=*/gc.r, wx, wy);
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }

    // Optional smoothing
    if(smoothing_iters_ > 0){
      smoothChaikin(path.poses, smoothing_iters_);
    }

    // Header + publish
    path.header.stamp = ros::Time::now();
    path.header.frame_id = world_frame_;  // use param (default "map")

    path_pub_.publish(path);
    ROS_INFO("Path with %zu poses published to /simple_planner/path", path.poses.size());

    // RViz markers (line + start/goal)
    publishMarkers(path);
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