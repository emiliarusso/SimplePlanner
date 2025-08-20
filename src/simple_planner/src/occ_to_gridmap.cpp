#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// Class that converts a ROS OccupancyGrid (from /map) into a GridMap
class OccToGridMap {
public:
  OccToGridMap() {
    ros::NodeHandle pnh("~");  // private NodeHandle 
    // Read parameters from ROS param server 
    pnh.param<std::string>("map_topic", map_topic_, "/map");          // topic of input OccupancyGrid
    pnh.param<std::string>("layer_name", layer_name_, "intensity");   // layer name in GridMap

    // Subscribe to OccupancyGrid topic (map)
    sub_ = nh_.subscribe(map_topic_, 1, &OccToGridMap::cb, this);

    // Publisher for GridMap messages
    pub_ = nh_.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  }

private:
  ros::NodeHandle nh_;       // standard node handle
  ros::Subscriber sub_;      // subscriber for OccupancyGrid
  ros::Publisher pub_;       // publisher for GridMap
  std::string map_topic_;    // input topic name
  std::string layer_name_;   // name of the data layer inside the GridMap

  // Callback: converts an OccupancyGrid into a GridMap
  void cb(const nav_msgs::OccupancyGrid& og) {
    // Create a GridMap with one layer (name = layer_name_)
    grid_map::GridMap gm({layer_name_});
    gm.setFrameId(og.header.frame_id);  // keep same reference frame (usually "map")

    // Compute dimensions of the map in meters
    double len_x = og.info.width  * og.info.resolution;
    double len_y = og.info.height * og.info.resolution;
    gm.setGeometry(grid_map::Length(len_x, len_y), og.info.resolution);

    // Compute map center position (world coordinates)
    double ox = og.info.origin.position.x;
    double oy = og.info.origin.position.y;
    double cx = ox + 0.5 * len_x;
    double cy = oy + 0.5 * len_y;
    gm.setPosition(grid_map::Position(cx, cy));

    // Reference to the layer data matrix
    auto& layer = gm[layer_name_];

    const int size_x = og.info.width;   // cells along X (width)
    const int size_y = og.info.height;  // cells along Y (height)

    // Iterate through all cells of the OccupancyGrid
    for (int x = 0; x < size_x; ++x) {
      for (int y = 0; y < size_y; ++y) {
        // Linear index in OccupancyGrid data (row-major order)
        const int i = y * size_x + x;
        const int8_t v = og.data[i];  // occupancy value: 0=free, 100=occupied, -1=unknown

        float val = 0.0f;
        if (v == 0)        val = 1.0f;  // free cell → 1.0
        else if (v == 100) val = 0.0f;  // occupied cell → 0.0
        else               val = 0.0f;  // unknown cell → treated as obstacle (0.0)

        // flip Y if the map appears upside down in RViz
        // int yy = size_y - 1 - y;  // use yy instead of y

        gm.at(layer_name_, grid_map::Index(x, y)) = val;
        
        // or
        // layer(x, y) = val;
      }
    }

    // Add timestamp
    gm.setTimestamp(ros::Time::now().toNSec());

    // Convert GridMap object into a ROS message
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(gm, msg);

    // Publish GridMap
    pub_.publish(msg);
  }
};

// Main entry point
int main(int argc, char** argv){
  ros::init(argc, argv, "occ_to_gridmap");  // initialize ROS node
  OccToGridMap node;                        // create instance of converter
  ros::spin();                              // keep running until shutdown
  return 0;
}