/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>
#include "markers/ManipulatorMarker.h"
#include "util/ros_conversions.h"

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
using OpenRAVE::IkSolverBasePtr;

using namespace or_rviz::util;

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

namespace or_rviz {
namespace markers {

OpenRAVE::Vector const ManipulatorMarker::kValidColor(0, 1, 0, 0.4);
OpenRAVE::Vector const ManipulatorMarker::kInvalidColor(1, 0, 0, 0.4);

ManipulatorMarker::ManipulatorMarker(InteractiveMarkerServerPtr server,
                                     ManipulatorPtr manipulator)
    : server_(server)
    , manipulator_(manipulator)
    , hidden_(false)
    , reset_pose_(false)
    , changed_pose_(true)
    , has_ik_(true)
    , force_update_(false)
    , current_pose_(manipulator->GetEndEffectorTransform())
{
    BOOST_ASSERT(server_);
    BOOST_ASSERT(manipulator);

    // Create the ghost manipulator.
    CreateGeometry();

    // Create a 6-DOF pose control.
    ik_marker_.header.frame_id = kDefaultWorldFrameId;
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

    CreateMenu();
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

bool ManipulatorMarker::is_hidden() const
{
    return hidden_;
}

void ManipulatorMarker::set_parent_frame(std::string const &frame_id)
{
    ik_marker_.header.frame_id = frame_id;
    force_update_ = true;

    for (LinkMarkerPtr const &link_marker : link_markers_ | map_values) {
        link_marker->set_parent_frame(frame_id);
    }

    for (JointMarkerPtr const &joint_marker : free_joint_markers_ | map_values) {
        joint_marker->set_parent_frame(frame_id);
    }
}

bool ManipulatorMarker::EnvironmentSync()
{
    ManipulatorPtr const manipulator = manipulator_;
    RobotBasePtr const robot = manipulator->GetRobot();
    IkSolverBasePtr const ik_solver = manipulator->GetIkSolver();
    RobotStateSaver const saver(robot, KinBody::Save_LinkTransformation);

    // Hack to avoid crashing if no IK solver is set.
    if (!ik_solver) {
        return false;
    }

    // Figure out what the free joints are.
    size_t const num_free = ik_solver->GetNumFreeParameters();
    std::vector<JointPtr> free_joints;
    InferFreeJoints(&free_joints);
    BOOST_ASSERT(free_joints.size() == num_free);

    std::vector<int> free_dof_indices;
    for (JointPtr const &free_joint : free_joints) {
        free_dof_indices.push_back(free_joint->GetJointIndex());
    }

    if (ik_solver) {
        RobotStateSaver const free_saver(robot, KinBody::Save_LinkTransformation);

        // Default to the current configuration of the robot.
        if (current_free_.size() != num_free) {
            robot->GetDOFValues(current_free_, free_dof_indices);
        }

        // Extract free parameters from the joint controls.
        bool changed_free = false;
        for (size_t ifree = 0; ifree < num_free; ++ifree) {
            JointPtr const &joint = free_joints[ifree];
            auto const it = free_joint_markers_.find(joint.get());
            if (it == free_joint_markers_.end()) {
                continue;
            }

            // Invalidate the IK solution if we change the free joint.
            JointMarkerPtr &joint_marker = it->second;
            if (joint_marker->angle() != current_free_[ifree]) {
                current_free_[ifree] = joint_marker->angle();
                changed_free = true;
            }
        }

        // Set and clamp these joint values to be within limits.
        robot->SetDOFValues(current_free_, KinBody::CLA_CheckLimitsSilent, free_dof_indices);
        robot->GetDOFValues(current_free_, free_dof_indices);
        
        // Update our IK solution.
        if (changed_pose_ || changed_free) {
            std::vector<OpenRAVE::dReal> new_ik;
            OpenRAVE::IkParameterization ik_param;
            ik_param.SetTransform6D(current_pose_);

            std::vector<std::vector<OpenRAVE::dReal> > ik_solutions;
            has_ik_ = manipulator_->FindIKSolution(ik_param, new_ik, 0);
            if (has_ik_) {
                current_ik_ = new_ik;
            }
        }
    }
    changed_pose_ = false;

    // Re-create the IK marker if necessary.
    if (force_update_) {
        server_->insert(ik_marker_);
        force_update_ = false;
    }

    // Update the pose of the ghost manipulator.
    auto const dof_indices = manipulator->GetArmIndices();
    robot->SetDOFValues(current_ik_, 1, dof_indices);

    bool is_changed = false;
    for (LinkMarkerPtr const &link_marker : link_markers_ | map_values) {
        bool const is_link_changed = link_marker->EnvironmentSync();
        if (!is_link_changed) {
            OpenRAVE::Transform const link_pose = link_marker->link()->GetTransform();
            link_marker->set_pose(link_pose);
        } else {
            UpdateMenu(link_marker);
        }

        // Set the color to indicate whether we have a valid IK solution.
        if (has_ik_) {
            link_marker->set_color(kValidColor);
        } else {
            link_marker->set_color(kInvalidColor);
        }

        is_changed = is_changed || is_link_changed;
    }

    if (ik_solver) {
        for (JointPtr const &joint : free_joints) {
            if (!joint) {
                continue;
            }

            // Lazily create the marker if it is missing.
            JointMarkerPtr &joint_marker = free_joint_markers_[joint.get()];
            if (!joint_marker) {
                joint_marker = boost::make_shared<JointMarker>(server_, joint);
            }

            // Update the pose of the control to match the ghost arm.
            OpenRAVE::Transform const joint_pose = JointMarker::GetJointPose(joint);
            joint_marker->set_pose(joint_pose);

            bool const is_joint_changed = joint_marker->EnvironmentSync();
            is_changed = is_changed || is_joint_changed;
        }
    }

    // Snap the pose handle to end-effector.
    if (reset_pose_) {
        server_->setPose(ik_marker_.name, toROSPose(current_pose_));
        reset_pose_ = false;
        is_changed = true;
    }

    return is_changed;
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

void ManipulatorMarker::CreateMenu()
{
    auto const cb = boost::bind(&ManipulatorMarker::MenuCallback, this, _1);

    menu_set_ = menu_handler_.insert("Set DOF Values", cb);
    menu_reset_ = menu_handler_.insert("Restore DOF Values", cb);
    menu_hide_ = menu_handler_.insert("Hide IK Controls", cb);
}

void ManipulatorMarker::UpdateMenu()
{
    for (LinkMarkerPtr const &link_marker : link_markers_ | map_values) {
        UpdateMenu(link_marker);
    }
}

void ManipulatorMarker::UpdateMenu(LinkMarkerPtr link_marker)
{
    menu_handler_.apply(*server_, link_marker->interactive_marker()->name);
}

void ManipulatorMarker::MenuCallback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    ManipulatorPtr const manipulator = manipulator_;
    RobotBasePtr const robot = manipulator->GetRobot();
    std::vector<int> const &arm_indices = manipulator->GetArmIndices();

    if (feedback->menu_entry_id == menu_set_) {
        robot->SetDOFValues(current_ik_, KinBody::CLA_CheckLimits, arm_indices);
        RAVELOG_DEBUG("Set manipulator '%s' to IK solution.\n",
            manipulator->GetName().c_str()
        );
    } else if (feedback->menu_entry_id == menu_reset_) {
        robot->GetDOFValues(current_ik_, arm_indices);

        reset_pose_ = true;
        current_pose_ = manipulator_->GetEndEffectorTransform();
        current_free_.clear();

        RAVELOG_DEBUG("Snapped to current configuration of manipulator '%s'.\n",
            manipulator->GetName().c_str()
        );
    } else if (feedback->menu_entry_id == menu_hide_) {
        hidden_ = true;
        RAVELOG_DEBUG("Disabling IK controls for manipulator '%s'.\n",
            manipulator->GetName().c_str()
        );
    }
}

void ManipulatorMarker::IkFeedback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
        current_pose_ = toORPose<OpenRAVE::dReal>(feedback->pose);
        changed_pose_ = true;
    }
}

void ManipulatorMarker::InferFreeJoints(std::vector<JointPtr> *free_joints) const
{
    static OpenRAVE::dReal const kEpsilon = 1e-3;

    ManipulatorPtr const manipulator = manipulator_;
    RobotBasePtr const robot = manipulator->GetRobot();
    IkSolverBasePtr const ik_solver = manipulator->GetIkSolver();

    std::vector<OpenRAVE::dReal> lower_limits, upper_limits;
    robot->GetDOFLimits(lower_limits, upper_limits);

    // We'll only accept a joint if it changes the value by at least kEpsilon.
    size_t const num_free = ik_solver->GetNumFreeParameters();
    std::vector<OpenRAVE::dReal> best_ratio(num_free, kEpsilon);
    std::vector<JointPtr> &best_joints = *free_joints;
    best_joints.assign(num_free, JointPtr());

    // Calculate the sensitivity of the free parameters to each joint.
    std::vector<int> const &arm_indices = manipulator->GetArmIndices();
    std::vector<OpenRAVE::dReal> free_motion;
    for (int const dof_index : arm_indices) {
        RobotStateSaver const saver(robot, KinBody::Save_LinkTransformation);
        std::vector<OpenRAVE::dReal> dof_values;
        robot->GetDOFValues(dof_values);

        // Move the joint to its limits.
        std::vector<OpenRAVE::dReal> lower_free;
        dof_values[dof_index] = lower_limits[dof_index];
        robot->SetDOFValues(dof_values);
        ik_solver->GetFreeParameters(lower_free);
        BOOST_ASSERT(lower_free.size() == num_free);

        std::vector<OpenRAVE::dReal> upper_free;
        dof_values[dof_index] = upper_limits[dof_index];
        robot->SetDOFValues(dof_values);
        ik_solver->GetFreeParameters(upper_free);
        BOOST_ASSERT(upper_free.size() == num_free);

        // Calculate the relative change in free parameters.
        for (size_t ifree = 0; ifree < num_free; ++ifree) {
            double const delta_param = upper_free[ifree] - lower_free[ifree];
            double const delta_value = upper_limits[ifree] - lower_limits[ifree];
            BOOST_ASSERT(delta_value > 0);
            double const ratio = std::fabs(delta_param / delta_value);

            if (ratio > best_ratio[ifree]) {
                best_ratio[ifree] = ratio;
                best_joints[ifree] = robot->GetJointFromDOFIndex(dof_index);
            }
        }
    }
}

}
}
