#include <boost/format.hpp>
#include <openrave/openrave.h>
#include <openrave/geometry.h>
#include <interactive_markers/interactive_marker_server.h>
#include "JointMarker.h"
#include "or_conversions.h"

using boost::format;
using boost::str;
using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

namespace or_interactivemarker {

JointMarker::JointMarker(InteractiveMarkerServerPtr server, JointPtr joint)
    : server_(server)
    , joint_(joint)
{
    BOOST_ASSERT(joint);

    // TODO: Support more types of joints.
    if (joint->GetType() != OpenRAVE::KinBody::JointRevolute) {
        return;
    }
    
    // Ignore static and mimic joints since we can't directly control them.
    if (joint->IsStatic()) {
        return;
    } else if (joint->IsMimic()) {
        return;
    }

    OpenRAVE::Transform pose = OpenRAVE::geometry::transformLookat(
        OpenRAVE::Vector(0, 0, 0), joint->GetAxis(), OpenRAVE::Vector(1, 0, 0));
    pose.trans = joint->GetAnchor();

    marker_.header.frame_id = kWorldFrameId;
    marker_.name = id();
    marker_.description = "";
    marker_.pose = toROSPose(pose);
    // TODO: Infer a good scale for the control.
    marker_.scale = 0.25;

    InteractiveMarkerControl control;
    // TODO: Why isn't this rotation about the x-axis?
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back(control);

    server_->insert(marker_);

#if 0
    server_->setCallback(ik_marker_.name,
        boost::bind(&ManipulatorMarker::IkFeedback, this, _1));
#endif
}

JointMarker::~JointMarker()
{
    server_->erase(marker_.name);
}

std::string JointMarker::id() const
{
    OpenRAVE::KinBodyPtr const body = joint_->GetParent();
    OpenRAVE::EnvironmentBasePtr const env = body->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    return str(format("Environment[%d].KinBody[%s].Joint[%s]")
               % environment_id % body->GetName() % joint_->GetName());
}

void JointMarker::EnvironmentSync()
{
}

}
