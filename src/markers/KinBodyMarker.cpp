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
#include "markers/KinBodyMarker.h"
#include "util/ros_conversions.h"

using boost::ref;
using boost::format;
using boost::str;
using boost::algorithm::ends_with;
using boost::adaptors::map_values;
using OpenRAVE::dReal;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::KinBodyWeakPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBaseWeakPtr;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using interactive_markers::InteractiveMarkerServer;
using interactive_markers::MenuHandler;

using namespace or_rviz::util;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef MenuHandler::EntryHandle EntryHandle;

namespace or_rviz {
namespace markers {

// TODO: Move this to a helper header.
static MenuHandler::CheckState BoolToCheckState(bool const &flag)
{
    if (flag) {
        return MenuHandler::CHECKED;
    } else {
        return MenuHandler::UNCHECKED;
    }
}

static bool CheckStateToBool(MenuHandler::CheckState const &state)
{
    return state == MenuHandler::CHECKED;
}


KinBodyMarker::KinBodyMarker(InteractiveMarkerServerPtr server,
                             KinBodyPtr kinbody)
    : server_(server)
    , kinbody_(kinbody)
    , robot_(boost::dynamic_pointer_cast<RobotBase>(kinbody))
    , parent_frame_id_(kDefaultWorldFrameId)
    , has_pose_controls_(false)
    , has_joint_controls_(false)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(kinbody);

    // Create the pose controls.
    interactive_marker_ = boost::make_shared<InteractiveMarker>();
    interactive_marker_->header.frame_id = kDefaultWorldFrameId;
    interactive_marker_->name = id();
    interactive_marker_->description = "";
    interactive_marker_->pose = toROSPose(kinbody->GetTransform());
    interactive_marker_->scale = 1.0; // TODO: Infer a good scale.

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_->controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_->controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_->controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_->controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_->controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_->controls.push_back(control);

    handle_kinbody_ = kinbody->RegisterChangeCallback(
        OpenRAVE::KinBody::Prop_Name,
        boost::bind(&KinBodyMarker::InvalidateKinBody, this)
    );
    handle_links_ = kinbody->RegisterChangeCallback(
          OpenRAVE::KinBody::Prop_LinkDraw
        | OpenRAVE::KinBody::Prop_LinkGeometry
        | OpenRAVE::KinBody::Prop_LinkEnable,
        boost::bind(&KinBodyMarker::InvalidateLinks, this)
    );
    handle_manipulators_ = kinbody->RegisterChangeCallback(
          OpenRAVE::KinBody::Prop_RobotManipulatorName
        | OpenRAVE::KinBody::Prop_RobotManipulatorSolver,
        boost::bind(&KinBodyMarker::InvalidateManipulators, this)
    );
}

KinBodyMarker::~KinBodyMarker()
{
    if (has_pose_controls_) {
        server_->erase(interactive_marker_->name);
    }
}

std::string KinBodyMarker::id() const
{
    KinBodyPtr const body = kinbody_.lock();
    EnvironmentBasePtr const env = body->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);
    return str(format("Environment[%d].KinBody[%s]")
               % environment_id % body->GetName());
}

void KinBodyMarker::set_parent_frame(std::string const &frame_id)
{
    if (frame_id == parent_frame_id_) {
        return; // no change
    }

    parent_frame_id_ = frame_id;
    interactive_marker_->header.frame_id = frame_id;

    if (has_pose_controls_) {
        server_->insert(*interactive_marker_);
    }

    for (LinkMarkerWrapper const &link_wrapper: link_markers_ | map_values) {
        link_wrapper.link_marker->set_parent_frame(frame_id);
    }
    
    for (KinBodyJointMarkerPtr const &joint_marker : joint_markers_ | map_values) {
        joint_marker->set_parent_frame(frame_id);
    }

    for (ManipulatorMarkerPtr const &manip_marker : manipulator_markers_ | map_values) {
        manip_marker->set_parent_frame(frame_id);
    }
}

std::vector<std::string> KinBodyMarker::group_names() const
{
    std::set<std::string> all_group_names;

    for (LinkMarkerWrapper const &wrapper: link_markers_ | map_values) {
        std::vector<std::string> const link_group_names = wrapper.link_marker->group_names();
        all_group_names.insert(link_group_names.begin(), link_group_names.end());
    }

    return std::vector<std::string>(all_group_names.begin(), all_group_names.end());
}

void KinBodyMarker::SwitchGeometryGroup(std::string const &group)
{
    // This is theoretically more efficient than calling SetGeometriesFromGroup
    // on each link. See the OpenRAVE documentation for more information.
    kinbody_.lock()->SetLinkGeometriesFromGroup(group);
}

void KinBodyMarker::AddMenuEntry(std::string const &name,
                                 boost::function<void ()> const &callback)
{
    CustomMenuEntry entry;
    entry.name = name;
    entry.callback = callback;
    menu_custom_kinbody_.push_back(entry);

    RAVELOG_DEBUG("Added menu option [ KinBody > %s ] to KinBody '%s'.\n",
        name.c_str(), kinbody_.lock()->GetName().c_str()
    );

    // TODO: Only re-generate the menu.
    link_markers_.clear();
}

void KinBodyMarker::AddMenuEntry(LinkPtr link,
                                 std::string const &name,
                                 boost::function<void ()> const &callback)
{
    CustomMenuEntry entry;
    entry.name = name;
    entry.callback = callback;

    menu_custom_links_[link.get()].push_back(entry);

    RAVELOG_DEBUG("Added menu option [ Link > %s ] to KinBody '%s' link '%s'.\n",
        name.c_str(), kinbody_.lock()->GetName().c_str(), link->GetName().c_str()
    );

    // TODO: Only re-generate the menu.
    link_markers_.clear();
}

void KinBodyMarker::AddMenuEntry(ManipulatorPtr manipulator,
                                 std::string const &name,
                                 boost::function<void ()> const &callback)
{
    CustomMenuEntry entry;
    entry.name = name;
    entry.callback = callback;
    menu_custom_manipulators_[manipulator.get()].push_back(entry);

    RAVELOG_DEBUG("Added menu option [ Manipulator > %s ] to KinBody '%s' manipulator '%s'.\n",
        name.c_str(), kinbody_.lock()->GetName().c_str(), manipulator->GetName().c_str()
    );

    // TODO: Only re-generate the menu.
    link_markers_.clear();
}

void KinBodyMarker::EnvironmentSync()
{
    typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
    typedef OpenRAVE::KinBody::JointPtr JointPtr;

    KinBodyPtr const kinbody = kinbody_.lock();

    // Update the KinBody's marker.
    if (has_pose_controls_) {
        OpenRAVE::Transform const kinbody_pose = kinbody->GetTransform();
        server_->setPose(interactive_marker_->name, toROSPose(kinbody_pose),
                         interactive_marker_->header);
    }

    // Update links. This includes the geometry of the KinBody.
    for (LinkPtr link : kinbody->GetLinks()) {
        LinkMarkerWrapper &wrapper = link_markers_[link.get()];
        KinBodyLinkMarkerPtr &link_marker = wrapper.link_marker;
        if (!link_marker) {
            link_marker = boost::make_shared<KinBodyLinkMarker>(server_, link);
            link_marker->set_parent_frame(parent_frame_id_);
            CreateMenu(wrapper);
            UpdateMenu(wrapper);
        }
        link_marker->EnvironmentSync();
    }

    // Update joints.
    for (JointPtr joint : kinbody->GetJoints()) {
        auto const it = joint_markers_.find(joint.get());
        if (it == joint_markers_.end()) {
            continue;
        }
        KinBodyJointMarkerPtr &joint_marker = it->second;

        // Lazily construct a new marker if necessary.
        if (!joint_marker) {
            joint_marker = boost::make_shared<KinBodyJointMarker>(server_, joint);
            joint_marker->set_parent_frame(parent_frame_id_);
        }
        joint_marker->EnvironmentSync();
    }

    // Update manipulators.
    typedef boost::unordered_map<
        OpenRAVE::RobotBase::Manipulator *, ManipulatorMarkerPtr
            >::iterator ManipulatorMarkerIterator;

    ManipulatorMarkerIterator it = manipulator_markers_.begin();
    bool manipulators_changed = false;

    while (it != manipulator_markers_.end()) {
        ManipulatorMarkerPtr const &manipulator_marker = it->second;

        if (!manipulator_marker->is_hidden()) {
            manipulator_marker->EnvironmentSync();
            ++it;
        }
        // The ghost manipulator was hidden, e.g. in C++ or using an internal
        // menu option. Remove the ManipulatorMarkerPtr and force the menu
        // to update (to un-check the IK option).
        else {
            it = manipulator_markers_.erase(it);
            manipulators_changed = true;
        }
    }

    if (manipulators_changed) {
        UpdateMenu();
    }
}

void KinBodyMarker::CreateMenu(LinkMarkerWrapper &link_wrapper)
{
    typedef boost::optional<EntryHandle> Opt;

    BOOST_ASSERT(!link_wrapper.has_menu);
    auto const cb = boost::bind(&KinBodyMarker::MenuCallback, this,
                                ref(link_wrapper), _1);
    MenuHandler &menu_handler = link_wrapper.link_marker->menu_handler();

    // KinBody controls.
    {
        EntryHandle parent = menu_handler.insert("Body");
        link_wrapper.menu_parent = Opt(parent);
        link_wrapper.menu_enabled = Opt(menu_handler.insert(parent, "Enabled", cb));
        link_wrapper.menu_visible = Opt(menu_handler.insert(parent, "Visible", cb));
        link_wrapper.menu_move = Opt(menu_handler.insert(parent, "Pose Controls", cb));
        link_wrapper.menu_joints = Opt(menu_handler.insert(parent, "Joint Controls", cb));

        EntryHandle geom_parent = menu_handler.insert(parent, "Geometry");
        link_wrapper.menu_geometry = Opt(geom_parent);
        link_wrapper.menu_geometry_visual = Opt(menu_handler.insert(geom_parent, "Visual", cb));
        link_wrapper.menu_geometry_collision = Opt(menu_handler.insert(geom_parent, "Collision", cb));
        link_wrapper.menu_geometry_both = Opt(menu_handler.insert(geom_parent, "Both", cb));

        // Geometry groups.
        EntryHandle groups_parent = menu_handler.insert(parent, "Geometry Groups");
        link_wrapper.menu_groups = Opt(groups_parent);

        link_wrapper.menu_groups_entries.clear();
        for (std::string const &group_name : group_names()) {
            // TODO: Add a callback.
            auto const group_cb = boost::bind(&KinBodyMarker::SwitchGeometryGroup, this, group_name);
            EntryHandle const group_handle = menu_handler.insert(groups_parent, group_name, group_cb);
            link_wrapper.menu_groups_entries[group_name] = group_handle;
        }

        // Custom KinBody entries.
        for (CustomMenuEntry const &menu_entry : menu_custom_kinbody_) {
            auto const custom_cb = boost::bind(menu_entry.callback);
            menu_handler.insert(parent, menu_entry.name, custom_cb);
        }
    }

    // Custom link entries.
    {
        LinkPtr const link = link_wrapper.link_marker->link();
        std::vector<CustomMenuEntry> const &entries = menu_custom_links_[link.get()];
        for (CustomMenuEntry const &menu_entry : entries) {
            // TODO: Add a custom callback.
            // TODO: Put these in the "Link" submenu.
            menu_handler.insert("LINK: " + menu_entry.name);
        }
    }

    // Manipulator controls. For now, we'll only add these options to links
    // that unambiguously belong to one manipulator.
    // TODO: Move this elsewhere.
    std::vector<ManipulatorPtr> manipulators;
    GetManipulators(link_wrapper.link_marker->link(), &manipulators);
    if (manipulators.size() == 1) {
        link_wrapper.parent_manipulator = manipulators.front();
    }
    
    if (link_wrapper.parent_manipulator) {
        EntryHandle parent = menu_handler.insert("Manipulator");
        link_wrapper.menu_manipulator = Opt(parent);
        // TODO: Implement joint control on the ghost manipulator.
        //link_wrapper.menu_manipulator_joints = Opt(menu_handler.insert(parent, "Joint Controls", cb));
        link_wrapper.menu_manipulator_active = Opt(menu_handler.insert(parent, "Set Active", cb));
        link_wrapper.menu_manipulator_ik = Opt(menu_handler.insert(parent, "Inverse Kinematics", cb));

        // Custom manipulator entries.
        std::vector<CustomMenuEntry> const &entries
            = menu_custom_manipulators_[link_wrapper.parent_manipulator.get()];
        for (CustomMenuEntry const &menu_entry : entries) {
            // TODO: Add a custom callback.
            menu_handler.insert(parent, menu_entry.name);
        }
    }
    link_wrapper.has_menu = true;
}

void KinBodyMarker::UpdateMenu()
{
    for (LinkMarkerWrapper &marker_wrapper : link_markers_ | map_values) {
        UpdateMenu(marker_wrapper);
        marker_wrapper.link_marker->UpdateMenu();
    }
}

void KinBodyMarker::UpdateMenu(LinkMarkerWrapper &link_wrapper)
{
    if (!link_wrapper.has_menu) {
        return;
    }

    MenuHandler &menu_handler = link_wrapper.link_marker->menu_handler();
    LinkPtr const link = link_wrapper.link_marker->link();

    menu_handler.setCheckState(*link_wrapper.menu_enabled,
        BoolToCheckState(link->IsEnabled()));
    menu_handler.setCheckState(*link_wrapper.menu_visible,
        BoolToCheckState(link->IsVisible()));
    menu_handler.setCheckState(*link_wrapper.menu_move,
        BoolToCheckState(has_pose_controls_));
    menu_handler.setCheckState(*link_wrapper.menu_joints,
        BoolToCheckState(has_joint_controls_));

    if (link_wrapper.menu_manipulator_ik) {
        bool const has_ghost = HasGhostManipulator(link_wrapper.parent_manipulator);
        menu_handler.setCheckState(*link_wrapper.menu_manipulator_ik,
            BoolToCheckState(has_ghost));
    }
}

void KinBodyMarker::MenuCallback(LinkMarkerWrapper &link_wrapper,
                                 InteractiveMarkerFeedbackConstPtr const &feedback)
{
    MenuHandler &menu_handler = link_wrapper.link_marker->menu_handler();
    KinBodyPtr kinbody = kinbody_.lock();

    // Toggle kinbody collision checking.
    if (feedback->menu_entry_id == *link_wrapper.menu_enabled) {
        MenuHandler::CheckState enabled_state;
        menu_handler.getCheckState(*link_wrapper.menu_enabled, enabled_state);
        bool const is_enabled = !CheckStateToBool(enabled_state);
        kinbody->Enable(is_enabled);
        RAVELOG_DEBUG("Toggled enable to %d for '%s'.\n",
            is_enabled, kinbody->GetName().c_str()
        );
    }
    // Toggle kinbody visibility.
    else if (feedback->menu_entry_id == *link_wrapper.menu_visible) {
        MenuHandler::CheckState visible_state;
        menu_handler.getCheckState(*link_wrapper.menu_visible, visible_state);
        bool const is_visible = !CheckStateToBool(visible_state);
        kinbody->SetVisible(is_visible);
        RAVELOG_DEBUG("Toggled visible to %d for '%s'.\n",
            is_visible, kinbody->GetName().c_str()
        );
    }
    // Change geometry visibility.
    else if (feedback->menu_entry_id == *link_wrapper.menu_geometry_visual) {
        for (LinkMarkerWrapper &link_wrapper : link_markers_ | map_values) {
            link_wrapper.link_marker->set_view_visual(true);
            link_wrapper.link_marker->set_view_collision(false);
        }
    }
    else if (feedback->menu_entry_id == *link_wrapper.menu_geometry_collision) {
        for (LinkMarkerWrapper &link_wrapper : link_markers_ | map_values) {
            link_wrapper.link_marker->set_view_visual(false);
            link_wrapper.link_marker->set_view_collision(true);
        }
    }
    else if (feedback->menu_entry_id == *link_wrapper.menu_geometry_both) {
        for (LinkMarkerWrapper &link_wrapper : link_markers_ | map_values) {
            link_wrapper.link_marker->set_view_visual(true);
            link_wrapper.link_marker->set_view_collision(true);
        }
    }
    // Toggle movement handles.
    else if (feedback->menu_entry_id == *link_wrapper.menu_move) {
        MenuHandler::CheckState move_state;
        menu_handler.getCheckState(*link_wrapper.menu_move, move_state);
        has_pose_controls_ = !CheckStateToBool(move_state);

        EnablePoseControls(has_pose_controls_);

        RAVELOG_DEBUG("Toggled pose controls to %d for '%s'.\n",
            has_pose_controls_, kinbody->GetName().c_str()
        );
    }
    // Toggle full-KinBody joint controls.
    else if (feedback->menu_entry_id == *link_wrapper.menu_joints) {
        MenuHandler::CheckState joints_state;
        menu_handler.getCheckState(*link_wrapper.menu_joints, joints_state);
        has_joint_controls_ = !CheckStateToBool(joints_state);

        if (!has_joint_controls_) {
            joint_markers_.clear();
        }
        // Allocate space for all joint controls. The actual controls will be
        // lazily created in the next EnvironmentSync callback.
        else {
            for (JointPtr const &joint : kinbody->GetJoints()) {
                // Accessing an element causes it to be default-constructed.
                joint_markers_[joint.get()];
            }
        }

        RAVELOG_DEBUG("Toggled joint controls to %d for '%s'.\n",
            has_joint_controls_, kinbody->GetName().c_str()
        );
    }
    // Set the manipulator as active.
    else if (link_wrapper.menu_manipulator_active
             && feedback->menu_entry_id == link_wrapper.menu_manipulator_active) {
        ManipulatorPtr const manipulator = link_wrapper.parent_manipulator;
        manipulator->GetRobot()->SetActiveManipulator(manipulator);
        RAVELOG_DEBUG("Set manipulator '%s' active for '%s'.\n",
            manipulator->GetName().c_str(), kinbody->GetName().c_str()
        );
    }
    // Toggle manipulator IK control.
    else if (link_wrapper.menu_manipulator_ik
             && feedback->menu_entry_id == link_wrapper.menu_manipulator_ik) {
        ManipulatorPtr const manipulator = link_wrapper.parent_manipulator;
        BOOST_ASSERT(manipulator);

        bool const ik_enabled = !HasGhostManipulator(manipulator);
        if (ik_enabled) {
            ManipulatorMarkerPtr &manipulator_marker = manipulator_markers_[manipulator.get()];
            if (!manipulator_marker) {
                manipulator_marker = boost::make_shared<ManipulatorMarker>(server_, manipulator);
                manipulator_marker->set_parent_frame(parent_frame_id_);
            }
        } else {
            manipulator_markers_.erase(manipulator.get());
        }
        RAVELOG_DEBUG("Toggled IK control to %d for '%s' manipulator '%s'.\n",
            ik_enabled, kinbody->GetName().c_str(),
            manipulator->GetName().c_str()
        );
    }

    UpdateMenu();
}

void KinBodyMarker::EnablePoseControls(bool enabled)
{
    if (enabled) {
        server_->insert(*interactive_marker_);
        server_->setCallback(interactive_marker_->name,
            boost::bind(&KinBodyMarker::PoseCallback, this, _1));
    } else {
        server_->erase(interactive_marker_->name);
    }
}

void KinBodyMarker::PoseCallback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    OpenRAVE::KinBodyPtr const kinbody = kinbody_.lock();

    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
        OpenRAVE::Transform const pose = toORPose<dReal>(feedback->pose);
        kinbody->SetTransform(pose);
    }
}

void KinBodyMarker::InvalidateKinBody()
{
    // We don't currently need to do anything when the KinBody's name changes.
}

void KinBodyMarker::InvalidateLinks()
{
    for (LinkMarkerWrapper const &link_wrapper : link_markers_ | map_values) {
        link_wrapper.link_marker->Invalidate();
    }
}

void KinBodyMarker::InvalidateManipulators()
{
    // The IK solver may have changed, so we have to completely re-construct
    // the manipulator markers.
    manipulator_markers_.clear();
}

bool KinBodyMarker::HasGhostManipulator(ManipulatorPtr const manipulator) const
{
    auto const it = manipulator_markers_.find(manipulator.get());
    return it != manipulator_markers_.end();
}

void KinBodyMarker::GetManipulators(
        LinkPtr link, std::vector<ManipulatorPtr> *manipulators) const
{
    BOOST_ASSERT(link);
    BOOST_ASSERT(manipulators);

    // Only robots have manipulators.
    KinBodyPtr const body = link->GetParent();
    auto const robot = boost::dynamic_pointer_cast<RobotBase>(body);
    if (!robot) {
        return;
    }

    for (ManipulatorPtr const &manipulator : robot->GetManipulators()) {
        // Check if this link is in the manipulator chain.
        LinkPtr const base_link = manipulator->GetBase();
        LinkPtr const tip_link = manipulator->GetEndEffector();
        std::vector<LinkPtr> chain_links;
        bool const success = robot->GetChain(base_link->GetIndex(), tip_link->GetIndex(), chain_links);

        if(!success)
        {
            continue;
        }

        auto const chain_it = std::find(chain_links.begin(), chain_links.end(), link);
        if (chain_it != chain_links.end()) {
            manipulators->push_back(manipulator);
            continue;
        }

        // Check if this link is a child (i.e. part of the end-effector).
        // TODO: This is necessary because IsChildLink is broken.
        std::vector<LinkPtr> child_links;
        manipulator->GetChildLinks(child_links);

        auto const child_it = std::find(child_links.begin(), child_links.end(), link);
        if (child_it != child_links.end()) {
            manipulators->push_back(manipulator);
            continue;
        }
    }
}

}
}
