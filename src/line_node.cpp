#include "ros-grid-map-line-itr/line.hpp"

int main(int argc, char ** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "line_node");

    // Instantiate line class
    grid_map_line::line node;

    // Start node
    node.start();

    return 0;
}