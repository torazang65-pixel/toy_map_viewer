#pragma once

#include "viewer/StaticLayer.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/InteractiveMarker.h>

#include <map>
#include <string>
#include <vector>

class LaneStaticLayer : public StaticLayer {
public:
    LaneStaticLayer(const std::string& name, ros::NodeHandle& nh,
                    interactive_markers::InteractiveMarkerServer& server,
                    interactive_markers::MenuHandler& menu_handler);

    void initMenu();
    void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) override;
    void clear() override;

    void applyMenuState();

private:
    struct LaneProp {
        bool explicit_lane;
        std_msgs::ColorRGBA original_color;
        visualization_msgs::InteractiveMarker int_marker;
    };

    void processLaneFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void processFile(const std::string& path, ros::Publisher& pub, int seq_idx,
                     double off_x, double off_y, double off_z);
    void updateMarkerVisual(const std::string& marker_name);
    void clearMarkers();
    std_msgs::ColorRGBA generateColor(int id);

    ros::NodeHandle nh_;
    interactive_markers::InteractiveMarkerServer& server_;
    interactive_markers::MenuHandler& menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_handle_lane_;

    std::vector<ros::Publisher> lane_publishers_;
    std::map<std::string, LaneProp> lane_properties_;
};
