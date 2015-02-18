#include "util/ros_conversions.h"
#include "util/InteractiveMarkerGraphHandle.h"

using visualization_msgs::InteractiveMarkerPtr;

namespace or_rviz {
namespace util {

InteractiveMarkerGraphHandle::InteractiveMarkerGraphHandle(
        InteractiveMarkerServerPtr const &interactive_marker_server,
        InteractiveMarkerPtr const &interactive_marker,
        boost::function<void (InteractiveMarkerGraphHandle *)> const &callback)
    : server_(interactive_marker_server)
    , interactive_marker_(interactive_marker)
    , remove_callback_(callback)
    , show_(true)
{
    BOOST_ASSERT(interactive_marker_server);
    BOOST_ASSERT(interactive_marker);

    server_->insert(*interactive_marker_);
}

InteractiveMarkerGraphHandle::~InteractiveMarkerGraphHandle()
{
    if (remove_callback_) {
        remove_callback_(this);
    }
    server_->erase(interactive_marker_->name);
}

void InteractiveMarkerGraphHandle::set_parent_frame(std::string const &frame_id)
{
    bool const is_changed = (frame_id != interactive_marker_->header.frame_id);
    interactive_marker_->header.frame_id = frame_id;

    if (show_ && is_changed) {
        server_->insert(*interactive_marker_);
    }
}

void InteractiveMarkerGraphHandle::SetTransform(
    OpenRAVE::RaveTransform<float> const &t)
{
    if (show_) {
        server_->setPose(interactive_marker_->name, toROSPose<>(t),
                         interactive_marker_->header);
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
