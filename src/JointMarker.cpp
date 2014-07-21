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
using OpenRAVE::KinBody;
using OpenRAVE::KinBodyPtr;

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

namespace or_interactivemarker {

JointMarker::JointMarker(InteractiveMarkerServerPtr server, JointPtr joint)
    : server_(server)
    , joint_(joint)
    , created_(false)
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

    marker_.header.frame_id = kWorldFrameId;
    marker_.name = id();
    marker_.description = "";
    marker_.pose = toROSPose(joint_pose());
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

    server_->setCallback(marker_.name,
        boost::bind(&JointMarker::JointCallback, this, _1));
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

OpenRAVE::Transform JointMarker::joint_pose() const
{
    OpenRAVE::Transform pose = OpenRAVE::geometry::transformLookat(
        OpenRAVE::Vector(0, 0, 0), joint_->GetAxis(), OpenRAVE::Vector(1, 0, 0));
    pose.trans = joint_->GetAnchor();
    return pose;
}

bool JointMarker::EnvironmentSync()
{
    if (created_) {
        // Update the pose of the marker.
        server_->setPose(marker_.name, toROSPose(joint_pose()));
    }
    created_ = true;
    return false;
}

void JointMarker::JointCallback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
        // Get the pose of the handle relative to the current joint position.
        // TODO: Why is this a rotation about the z-axis? It should be the y-axis.
        OpenRAVE::Transform const pose = joint_pose().inverse() * toORPose(feedback->pose);
        OpenRAVE::Vector const axis_angle = OpenRAVE::geometry::axisAngleFromQuat(pose.rot);
        double const delta_value = axis_angle[2];

        // Get the current joint value.
        KinBodyPtr const kinbody = joint_->GetParent();
        std::vector<int> dof_indices;
        std::vector<OpenRAVE::dReal> dof_values;
        dof_indices.push_back(joint_->GetJointIndex());
        kinbody->GetDOFValues(dof_values, dof_indices);

        // Update the joint value.
        // TODO: Why is the sign flipped?
        BOOST_ASSERT(joint_->GetDOF() == 1);
        BOOST_ASSERT(dof_values.size() == 1);
        dof_values.front() -= delta_value;
        kinbody->SetDOFValues(dof_values, KinBody::CLA_CheckLimitsSilent, dof_indices);
    }
}

}
