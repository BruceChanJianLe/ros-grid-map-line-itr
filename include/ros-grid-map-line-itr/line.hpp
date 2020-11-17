#ifndef __line_H_
#define __line_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/PolygonStamped.h>
// #include <grid_map_msgs/GridMap.h>

#include <vector>
#include <string>

namespace grid_map_line
{
    class line
    {
        private:
            // ROS Declaration
            ros::NodeHandle private_nh_;
            ros::Publisher grid_map_pub_;
            ros::Publisher polygon_pub_;

            // Private variables
            int rate_;
            grid_map::GridMap map_;

            // Private functions
            void publish();
            void process();
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