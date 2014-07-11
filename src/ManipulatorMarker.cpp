#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "ManipulatorMarker.h"
#include "or_conversions.h"

using boost::format;
using boost::str;
using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotBasePtr;

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

    // TODO: This is a hack to avoid creating ghost manipulators of ghost
    // manipulators; i.e. an infinite loop.
    RobotBasePtr const robot = manipulator->GetRobot();
    if (boost::starts_with(robot->GetName(), "Ghost")) {
        return;
    }

    // Create a ghost manipulator for visualization.
    ghost_manipulator_ = CreateGhost(manipulator);
    ghost_robot_ = ghost_manipulator_->GetRobot();
    BOOST_ASSERT(ghost_manipulator_ && ghost_robot_);

    ik_marker_.header.frame_id = kWorldFrameId;
    ik_marker_.name = id();
    ik_marker_.description = manipulator_->GetName();
    ik_marker_.pose = toROSPose(manipulator->GetEndEffectorTransform());
    ik_marker_.scale = 0.25;

    // Create a 6-DOF pose control.
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
    if (!ghost_manipulator_) {
        // WTF: How could this happen?
        return;
    }

    if (changed_pose_ && manipulator_->GetIkSolver()) {
        OpenRAVE::IkParameterization ik_param;
        ik_param.SetTransform6D(current_pose_);
    
        bool const solution = ghost_manipulator_->FindIKSolution(ik_param, current_ik_, 0);
        if (solution) {
            // TODO: Copy over other DOF values from the real robot.
            auto const dof_indices = ghost_manipulator_->GetArmIndices();
            ghost_robot_->SetDOFValues(current_ik_, 1, dof_indices);
        }
    }

    changed_pose_ = false;
}

void ManipulatorMarker::IkFeedback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
        current_pose_ = toORPose(feedback->pose);
        changed_pose_ = true;
    }
}

ManipulatorPtr ManipulatorMarker::CreateGhost(ManipulatorPtr manipulator)
{
    // Copy the LinkInfo structures.
    std::vector<LinkPtr> links;
    GetAllChildLinks(manipulator, &links);

    std::vector<LinkInfoConstPtr> link_infos;
    for (LinkPtr const &link : links) {
        // TODO: Need to deep-copy the GeometryInfoPtr's.
        LinkInfo const link_info = link->UpdateAndGetInfo();
        link_infos.push_back(boost::make_shared<LinkInfo>(link_info));
    }

    // Copy the JointInfo structures.
    std::vector<JointPtr> joints;
    GetJoints(links, &joints);

    std::vector<JointInfoConstPtr> joint_infos;
    for (JointPtr const &joint : joints) {
        JointInfo joint_info = joint->UpdateAndGetInfo();
        joint_infos.push_back(boost::make_shared<JointInfo>(joint_info));
    }

    // Copy the ManipulatorInfo structure.
    RobotBasePtr const real_robot = manipulator->GetRobot();
    ManipulatorInfo const manipulator_info = manipulator->GetInfo();
    std::vector<ManipulatorInfoConstPtr> manipulator_infos;
    manipulator_infos.push_back(boost::make_shared<ManipulatorInfo>(manipulator_info));

    // Don't attach any sensors.
    std::vector<AttachedSensorInfoConstPtr> sensor_infos;

    // Create the KinBody.
    EnvironmentBasePtr const env = real_robot->GetEnv();
    RobotBasePtr const ghost_robot = OpenRAVE::RaveCreateRobot(env, "");
    ghost_robot->Init(link_infos, joint_infos, manipulator_infos, sensor_infos);

    // Assign the ghost a unique name.
    std::string const name = str(format("Ghost.Robot[%s].Manipulator[%s]")
                                 % real_robot->GetName() % manipulator->GetName());
    ghost_robot->SetName(name);
    env->Add(ghost_robot, true);

    // Setup the new model.
    ghost_robot->Enable(false);
    ghost_robot->SetVisible(true);

    for (LinkPtr const link : ghost_robot->GetLinks()) {
        for (GeometryPtr const geom : link->GetGeometries()) {
            geom->SetTransparency(0.5);
        }
    }

    // Copy the IK solver.
    ManipulatorPtr const ghost_manipulator = ghost_robot->GetManipulator(manipulator->GetName());
    BOOST_ASSERT(ghost_manipulator);

    if (manipulator->GetIkSolver()) {
        RAVELOG_INFO("Setting IK solver.\n");
        ghost_manipulator->SetIkSolver(manipulator->GetIkSolver());
    }

    return ghost_manipulator;
}

void ManipulatorMarker::GetAllChildLinks(ManipulatorPtr manipulator,
                                         std::vector<LinkPtr> *children)
{
    BOOST_ASSERT(manipulator);
    BOOST_ASSERT(children);

    RobotBasePtr const robot = manipulator->GetRobot();
    LinkPtr const ee = manipulator->GetEndEffector();
    LinkPtr const base = manipulator->GetBase();

    // Get links in the kinematic chain
    std::vector<LinkPtr> chain_links;
    bool const connected = robot->GetChain(base->GetIndex(), ee->GetIndex(), chain_links);
    BOOST_ASSERT(connected);

    // Get links in the end-effector.
    std::vector<LinkPtr> child_links;
    manipulator->GetChildLinks(child_links);

    children->clear();
    children->insert(children->end(), chain_links.begin(), chain_links.end());
    children->insert(children->end(), child_links.begin(), child_links.end());

    // TODO: Why does this contain duplicates!? I have no idea how multiple
    // links can be in both the chain and the ned-effector.
    std::sort(children->begin(), children->end());
    auto const it = std::unique(children->begin(), children->end());
    children->erase(it, children->end());
}

void ManipulatorMarker::GetJoints(std::vector<LinkPtr> const &links,
                                  std::vector<JointPtr> *joints)
{
    BOOST_ASSERT(joints);

    // Build a set for efficient membership queries.
    std::set<LinkPtr> link_set(links.begin(), links.end());

    // Iterate over all joints and select any whose parent and child are both
    // in set of links. It would be more efficient to only look at the joints
    // attached to those links, but it's not possible to query the joints
    // attached to a link in OpenRAVE.
    RobotBasePtr const robot = manipulator_->GetRobot();
    std::vector<JointPtr> const &active_joints = robot->GetDependencyOrderedJoints();
    std::vector<JointPtr> const &passive_joints = robot->GetPassiveJoints();
    std::vector<JointPtr> all_joints;
    all_joints.insert(all_joints.end(), active_joints.begin(), active_joints.end());
    all_joints.insert(all_joints.end(), passive_joints.begin(), passive_joints.end());

    for (JointPtr const &joint : all_joints) {
        LinkPtr const link1 = joint->GetFirstAttached();
        LinkPtr const link2 = joint->GetSecondAttached();

        if (link_set.count(link1) && link_set.count(link2)) {
            joints->push_back(joint);
        }
    }
}

bool ManipulatorMarker::IsChildLink(LinkPtr const parent, LinkPtr const child)
{
    BOOST_ASSERT(parent);
    BOOST_ASSERT(child);

    LinkPtr curr_link = child;
    while (curr_link) {
        if (curr_link == parent) {
            return true;
        }

        std::vector<LinkPtr> parent_links;
        curr_link->GetParentLinks(parent_links);
        BOOST_ASSERT(parent_links.size() <= 1);

        if (!parent_links.empty()) {
            curr_link = parent_links.front();
        } else {
            curr_link = LinkPtr();
        }
    }
    return false;
}

}
