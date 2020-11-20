#include "ros-grid-map-line-itr/line.hpp"

namespace grid_map_line
{
    line::line()
    :   private_nh_("~"),
        grid_map_pub_(private_nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true)),
        polygon_pub_(private_nh_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true)),
        map_(std::vector<std::string>({"type"}))
    {
        // Load yaml ROS param
        private_nh_.param("rate", rate_, 5);

        // Define map_msg_
        map_.setGeometry(grid_map::Length(1.0, 1.0), 0.05, grid_map::Position(0.0, 0.0));
        map_.setFrameId("map");

        // Publish initial map
        publish();
    }


    line::~line()
    {
        ;
    }


    void line::publish()
    {
        // Print lenght
        ROS_INFO_STREAM(
            "map length x: " << map_.getLength().x() <<
            "\tmap length y: " << map_.getLength().y() <<
            "\n map position: " << map_.getPosition()
        );

        // Set msg timestamp
        map_.setTimestamp(ros::Time::now().toNSec());
        // Create map_msg
        grid_map_msgs::GridMap msg;
        // Convert grid map to grid map msg
        grid_map::GridMapRosConverter::toMessage(map_, msg);
        // Publish msg
        grid_map_pub_.publish(msg);

        // DEBUG
        ROS_INFO_STREAM("line node: grid map being published at" << msg.info.header.stamp.toSec());
    }


    void line::process()
    {
        ros::Rate r(rate_);

        // Refresh map
        map_.clearAll();
        // Publish map
        publish();

        // Define line index
        grid_map::Position a_point(0.3, 0.3);
        grid_map::Position b_point(-0.3, -0.3);
        grid_map::Index a;
        grid_map::Index b;
        if(grid_map::getIndexFromPosition(a, a_point, map_.getLength(), map_.getPosition(), map_.getResolution(), map_.getSize())
        && grid_map::getIndexFromPosition(b, b_point, map_.getLength(), map_.getPosition(), map_.getResolution(), map_.getSize()))
        {
            // ROS_INFO_STREAM("index at a:" << a.value << ", " << a.y );
            // Loop through line region
            for(grid_map::LineIterator itr(map_, a, b); !itr.isPastEnd(); ++itr)
            {
                // Change value from 0.0 to 1.0
                map_.at("type", *itr) = 1.0;
                // Publish map
                publish();
                // Sleep process
                r.sleep();
            }
        }
    }


    void line::start()
    {
        // Process
        process();
    }
} // namespace grid_map_line
