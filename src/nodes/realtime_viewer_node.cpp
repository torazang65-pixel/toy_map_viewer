#include <ros/ros.h>
#include <viewer/viewer_animation_loader.h>
#include <viewer/viewer_offset.h>
#include <viewer/viewer_static_map_loader.h>

class RealtimeViewerNode {
public:
    RealtimeViewerNode()
        : nh_("~"),
          static_map_loader_(nh_, offset_),
          animation_loader_(nh_, offset_) {}

    void run() {
        static_map_loader_.Publish();
        animation_loader_.Start();
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    realtime_line_generator::viewer::OffsetState offset_;
    realtime_line_generator::viewer::StaticMapLoader static_map_loader_;
    realtime_line_generator::viewer::AnimationLoader animation_loader_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_viewer_node", ros::init_options::AnonymousName);
    RealtimeViewerNode node;
    node.run();
    return 0;
}
