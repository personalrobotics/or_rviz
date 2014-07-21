#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>
#include "ManipulatorMarker.h"
#include "or_conversions.h"

using boost::format;
using boost::str;
using boost::adaptors::map_values;
using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::KinBody;

typedef OpenRAVE::RobotBase::RobotStateSaver RobotStateSaver;

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::LinkInfo LinkInfo;
typedef OpenRAVE::KinBody::LinkInfoConstPtr LinkInfoConstPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;
typedef OpenRAVE::KinBody::JointInfo JointInfo;
typedef OpenRAVE::KinBody::JointInfoConstPtr JointInfoConstPtr;
typedef OpenRAVE::RobotBase::ManipulatorInfo ManipulatorInfo;
typedef OpenRAVE::RobotBase::ManipulatorInfoConstPtr ManipulatorInfoConstPtr;
typedef OpenRAVE::RobotBase::AttachedSensorInfo AttachedSensorInfo;
typedef OpenRAVE::RobotBase::AttachedSensorInfoConstPtr AttachedSensorInfoConstPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";
static std::string const kGhostKey = "interactive_marker::ghost";

namespace or_interactivemarker {

ManipulatorMarker::ManipulatorMarker(InteractiveMarkerServerPtr server,
                                     ManipulatorPtr manipulator)
    : server_(server)
    , manipulator_(manipulator)
    , changed_pose_(true)
    , current_pose_(manipulator->GetEndEffectorTransform())
{
    BOOST_ASSERT(server_);
    BOOST_ASSERT(manipulator);

    // Create the ghost manipulator.
    CreateGeometry();

    // Create a 6-DOF pose control.
    ik_marker_.header.frame_id = kWorldFrameId;
    ik_marker_.name = id();
    ik_marker_.description = manipulator_->GetName();
    ik_marker_.pose = toROSPose(manipulator->GetEndEffectorTransform());
    ik_marker_.scale = 0.25;

    {
        InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        ik_marker_.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        ik_marker_.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        ik_marker_.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        ik_marker_.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        ik_marker_.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        ik_marker_.controls.push_back(control);
    }

    server_->insert(ik_marker_);
    server_->setCallback(ik_marker_.name,
        boost::bind(&ManipulatorMarker::IkFeedback, this, _1));
}

ManipulatorMarker::~ManipulatorMarker()
{
    server_->erase(ik_marker_.name);
}

std::string ManipulatorMarker::id() const
{
    OpenRAVE::RobotBasePtr const robot = manipulator_->GetRobot();
    OpenRAVE::EnvironmentBasePtr const env = robot->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    return str(format("Environment[%d].KinBody[%s].Manipulator[%s]")
               % environment_id % robot->GetName() % manipulator_->GetName());
}

void ManipulatorMarker::EnvironmentSync()
{
    ManipulatorPtr const manipulator = manipulator_;
    RobotBasePtr const robot = manipulator->GetRobot();
    RobotStateSaver saver(robot, KinBody::Save_LinkTransformation);

    // Update our IK solution.
    if (changed_pose_ && manipulator_->GetIkSolver()) {
        OpenRAVE::IkParameterization ik_param;
        ik_param.SetTransform6D(current_pose_);
        manipulator_->FindIKSolution(ik_param, current_ik_, 0);
    }
    changed_pose_ = false;

    // Update the pose of the ghost manipulator.
    auto const dof_indices = manipulator->GetArmIndices();
    robot->SetDOFValues(current_ik_, 1, dof_indices);

    for (LinkMarkerPtr const &link_marker : link_markers_ | map_values) {
        bool const is_link_changed = link_marker->EnvironmentSync();
        if (!is_link_changed) {
            OpenRAVE::Transform const link_pose = link_marker->link()->GetTransform();
            link_marker->set_pose(link_pose);
        }
    }
}

void ManipulatorMarker::CreateGeometry()
{
    link_markers_.clear();

    ManipulatorPtr const manipulator = manipulator_;
    RobotBasePtr const robot = manipulator->GetRobot();

    // Get links in the manipulator chain.
    std::vector<LinkPtr> chain_links;
    LinkPtr const base_link = manipulator->GetBase();
    LinkPtr const tip_link = manipulator->GetEndEffector();
    bool const success = robot->GetChain(
            base_link->GetIndex(), tip_link->GetIndex(), chain_links);
    BOOST_ASSERT(success);

    // Get end-effector links.
    std::vector<LinkPtr> child_links;
    manipulator->GetChildLinks(child_links);

    // Remove duplicates in case the end-effector link is double-counted.
    std::set<LinkPtr> links;
    links.insert(chain_links.begin(), chain_links.end());
    links.insert(child_links.begin(), child_links.end());

    // Render each link using a LinkMarker.
    for (LinkPtr const &link : links) {
        link_markers_[link.get()] = boost::make_shared<LinkMarker>(server_, link, true);
    }
}

void ManipulatorMarker::IkFeedback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
        current_pose_ = toORPose(feedback->pose);
        changed_pose_ = true;
    }
}

}
