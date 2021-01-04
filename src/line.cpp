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
    }


    void line::process()
    {
        // Refresh map
        map_.clearAll();
        // Publish map
        publish();

        // Catch Error
        try
        {
            // Check if points are valid (inside of map)
            if(map_.getIndex(points_["point_a"], index_a_)
            && map_.getIndex(points_["point_b"], index_b_))
            {
                // Loop through line region
                for(grid_map::LineIterator itr(map_, index_a_, index_b_); !itr.isPastEnd(); ++itr)
                {
                    // Change value from 0.0 to 1.0
                    map_.at("type", *itr) = 1.0;
                    // Publish map
                    publish();
                }
            }
            else
            {
                // Obtain closest point if it is invalid
                if(!map_.isInside(points_["point_a"]))
                    // points_["point_a"] = map_.getClosestPositionInMap(map_.getClosestPositionInMap(points_["point_a"]) + grid_map::Position(0.01, 0.01));
                    points_["point_a"] = map_.getClosestPositionInMap(points_["point_a"]);

                if(!map_.isInside(points_["point_b"]))
                    points_["point_b"] = map_.getClosestPositionInMap(points_["point_b"]);

                // Obtain index from closest point
                if(map_.getIndex(points_["point_a"], index_a_)
                && map_.getIndex(points_["point_b"], index_b_))
                {
                    // Loop through line region
                    for(grid_map::LineIterator itr(map_, index_a_, index_b_); !itr.isPastEnd(); ++itr)
                    {
                        // Change value from 0.0 to 1.0
                        map_.at("type", *itr) = 1.0;
                        // Publish map
                        publish();
                    }
                }
                else
                {
                    ROS_INFO_STREAM("Unable to obtain index correctly...");
                }

            }
        }
        catch(...)
        {
            ROS_INFO_STREAM("process function: ERROR!");
        }
        
    }


    void line::prepare_int_marker(const std::string name, const std::string description)
    {
        // Prepare point a
        int_marker_msg_.header.frame_id = map_.getFrameId();

        // Set interactive marker scale
        int_marker_msg_.scale = 0.15;

        if(name == "point_b")
        {
            int_marker_msg_.pose.position.x = 0.3;
        }

        // Prepare visual marker for interactive marker

            // Marker visual
            marker_msg_.type = visualization_msgs::Marker::CUBE;

            // marker scale (obtain from interactive marker)
            marker_msg_.scale.x = int_marker_msg_.scale * 0.30;
            marker_msg_.scale.y = int_marker_msg_.scale * 0.30;
            marker_msg_.scale.z = int_marker_msg_.scale * 0.30;

            // Marker color
            marker_msg_.color.r = 0.5;
            marker_msg_.color.g = 0.5;
            marker_msg_.color.b = 0.5;
            marker_msg_.color.a = 1.0;

            // Prepare visualization for interactive marker
            viz_int_marker_msg_.always_visible = true;
            viz_int_marker_msg_.markers.emplace_back(marker_msg_);

        // Planar Marker
        int_marker_msg_.name = name;
        int_marker_msg_.description = description;

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
        insert_int_marker(int_marker_msg_.name);

        // Add marker to points map
        points_[name] = grid_map::Position(0, 0);
    }


    void line::insert_int_marker(const std::string marker_name)
    {
        // Insert marker
        int_server_->insert(int_marker_msg_);

        // Prepare lambda function with swallow argument (marker name)
        auto int_server_callback =
            [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback, std::string name) -> void
            {
                // Pose update event
                if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
                {
                    // DEBUG
                    ROS_INFO_STREAM("DEBUG: marker name: " << name << " marker x: " << feedback->pose.position.x << " marker y: " << feedback->pose.position.y);

                    // Update the marker position to point position
                    this->points_[name] = grid_map::Position(feedback->pose.position.x, feedback->pose.position.y);

                    // Set update_ to true
                    this->update_ = true;
                }
            };

        // Set marker callback
        int_server_->setCallback(
            int_marker_msg_.name,
            [this, int_server_callback, marker_name](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
            {
                // Use callback function
                int_server_callback(feedback, marker_name);
            }
        );

        // Commit and apply changes
        int_server_->applyChanges();
    }


    void line::start()
    {
        ros::Rate r(5);

        // Prepare marker
        prepare_int_marker("point_a", "Point A");
        prepare_int_marker("point_b", "Point B");

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
