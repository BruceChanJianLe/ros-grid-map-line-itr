#ifndef __line_H_
#define __line_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/PolygonStamped.h>

// RViz interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <vector>
#include <string>
#include <memory>
#include <map>

namespace grid_map_line
{
    class line
    {
        private:
            // ROS Declaration
            ros::NodeHandle private_nh_;
            ros::Publisher grid_map_pub_;
            ros::Publisher polygon_pub_;

            // Interactive marker server
            std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_;
            visualization_msgs::InteractiveMarker int_marker_msg_;
            visualization_msgs::Marker marker_msg_;
            visualization_msgs::InteractiveMarkerControl viz_int_marker_msg_;
            visualization_msgs::InteractiveMarkerControl con_int_marker_msg_;

            // Private variables
            int rate_;
            grid_map::GridMap map_;
            bool update_;
            grid_map::Index index_a_;
            grid_map::Index index_b_;
            std::map<std::string, grid_map::Position> points_;

            // Private functions
            void publish();
            void process();
            void prepare_int_marker(const std::string,const std::string);
            void insert_int_marker(const std::string);
        public:
            // Constructor
            line();

            // Destructor
            ~line();

            // Node starter
            void start();
    };
} // namespace grid_map_line


#endif