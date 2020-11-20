#include "ros-grid-map-line-itr/line.hpp"

namespace grid_map_line
{
    line::line()
    :   private_nh_("~"),
        grid_map_pub_(private_nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true)),
        polygon_pub_(private_nh_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true)),
        map_(std::vector<std::string>({"type"})),
        int_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("int_server", "", false)),
        update_(true)
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
            }
        }
    }


    void line::prepare_int_marker()
    {
        // Prepare point a
        int_marker_msg_.header.frame_id = map_.getFrameId();

        // Set interactive marker scale
        int_marker_msg_.scale = 1;

        // Prepare visual marker for interactive marker

            // Marker visual
            marker_msg_.type = visualization_msgs::Marker::CUBE;

            // marker scale (obtain from interactive marker)
            marker_msg_.scale.x = int_marker_msg_.scale * 0.45;
            marker_msg_.scale.y = int_marker_msg_.scale * 0.45;
            marker_msg_.scale.z = int_marker_msg_.scale * 0.45;

            // Marker color
            marker_msg_.color.r = 0.5;
            marker_msg_.color.g = 0.5;
            marker_msg_.color.b = 0.5;
            marker_msg_.color.a = 1.0;

            // Prepare visualization for interactive marker
            viz_int_marker_msg_.always_visible = true;
            viz_int_marker_msg_.markers.emplace_back(marker_msg_);

        // Planar Marker
        int_marker_msg_.name = "Planar Marker A";
        int_marker_msg_.description = "point a";

        // Display control

            // Planar movement control
            con_int_marker_msg_.name = "planar_control";
            con_int_marker_msg_.orientation.x = 0;
            con_int_marker_msg_.orientation.y = 1;
            con_int_marker_msg_.orientation.z = 0;
            con_int_marker_msg_.orientation.w = 1;
            con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

                // Attach control to interactive marker
                int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

            // Prepare visualization marker to attach to control marker
            con_int_marker_msg_.markers.emplace_back(marker_msg_);
            con_int_marker_msg_.always_visible = true;

                // Attach control marker with visualize marker in it to interactive marker
                int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

        // Insert interactive marker into interactive marker server
        insert_int_marker();
    }


    void line::insert_int_marker()
    {
        // Insert marker
        int_server_->insert(int_marker_msg_);

        // Set marker callback
        int_server_->setCallback(
            int_marker_msg_.name,
            [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
            {
                // Pose update event
                if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
                {
                    // limit x position of interactive marker
                    if(feedback->pose.position.x > map_.getLength().x())
                    {
                        ;
                    }
                }
            }
        );
    }


    void line::start()
    {
        ros::Rate r(5);

        // Prepare marker

        // Insert marker

        while(private_nh_.ok())
        {
            if(update_)
            {
                // Process
                process();

                // Set update to false
                update_ = false;
            }

            // process callbacks
            ros::spinOnce();

            // Sleep Node
            r.sleep();
        }
    }
} // namespace grid_map_line
