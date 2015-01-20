#include "util/ros_conversions.h"
#include "util/InteractiveMarkerGraphHandle.h"

using visualization_msgs::InteractiveMarkerPtr;

namespace or_interactivemarker {
namespace util {

InteractiveMarkerGraphHandle::InteractiveMarkerGraphHandle(
        InteractiveMarkerServerPtr const &interactive_marker_server,
        InteractiveMarkerPtr const &interactive_marker)
    : server_(interactive_marker_server)
    , interactive_marker_(interactive_marker)
    , show_(true)
{
    BOOST_ASSERT(interactive_marker_server);
    BOOST_ASSERT(interactive_marker);

    server_->insert(*interactive_marker_);
}

InteractiveMarkerGraphHandle::~InteractiveMarkerGraphHandle()
{
    server_->erase(interactive_marker_->name);
}

void InteractiveMarkerGraphHandle::set_parent_frame(std::string const &frame_id)
{
    interactive_marker_->header.frame_id = frame_id;

    if (show_) {
        server_->insert(*interactive_marker_);
    }
}

void InteractiveMarkerGraphHandle::SetTransform(OpenRAVE::RaveTransform<float> const &t)
{
    if (show_) {
        server_->setPose(interactive_marker_->name, toROSPose<>(t));
    }
}

void InteractiveMarkerGraphHandle::SetShow(bool show)
{
    if (show && !show_) {
        server_->insert(*interactive_marker_);
    } else if (!show && show_) {
        server_->erase(interactive_marker_->name);
    }
    show_ = show;
}

}
}
