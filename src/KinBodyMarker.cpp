#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>
#include "KinBodyMarker.h"

using boost::ref;
using boost::format;
using boost::str;
using boost::algorithm::ends_with;
using boost::adaptors::map_values;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::KinBodyWeakPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBaseWeakPtr;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using interactive_markers::InteractiveMarkerServer;
using interactive_markers::MenuHandler;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef MenuHandler::EntryHandle EntryHandle;


namespace or_interactivemarker {

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
    , has_joint_controls_(false)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(kinbody);
}

KinBodyMarker::~KinBodyMarker()
{
}

void KinBodyMarker::EnvironmentSync()
{
    typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
    typedef OpenRAVE::KinBody::JointPtr JointPtr;

    KinBodyPtr const kinbody = kinbody_.lock();

    // Update links. This includes the geometry of the KinBody.
    for (LinkPtr link : kinbody->GetLinks()) {
        LinkMarkerWrapper &wrapper = link_markers_[link.get()];
        KinBodyLinkMarkerPtr &link_marker = wrapper.link_marker;
        if (!link_marker) {
            link_marker = boost::make_shared<KinBodyLinkMarker>(server_, link);
            CreateMenu(wrapper);
            UpdateMenu(wrapper);
        }
        link_marker->EnvironmentSync();
    }

    // Update joints.
    if (has_joint_controls_) {
        for (JointPtr joint : kinbody->GetJoints()) {
            JointMarkerPtr &joint_marker = joint_markers_[joint.get()];
            if (!joint_marker) {
                joint_marker = boost::make_shared<JointMarker>(server_, joint);
            }
            joint_marker->EnvironmentSync();
        }
    } else {
        joint_markers_.clear();
    }

    // Update manipulators.
    for (ManipulatorMarkerPtr const &manipulator_marker : manipulator_markers_ | map_values) {
        manipulator_marker->EnvironmentSync();
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
        link_wrapper.menu_joints = Opt(menu_handler.insert(parent, "Joint Controls", cb));
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
        link_wrapper.menu_manipulator_joints = Opt(menu_handler.insert(parent, "Joint Controls", cb));
        link_wrapper.menu_manipulator_ik = Opt(menu_handler.insert(parent, "Inverse Kinematics", cb));
    }
    link_wrapper.has_menu = true;
}

void KinBodyMarker::UpdateMenu()
{
    for (LinkMarkerWrapper &marker_wrapper : link_markers_ | map_values) {
        UpdateMenu(marker_wrapper);
        marker_wrapper.link_marker->UpdateMenu();
        // TODO: How can the link notify us that our menu changed?
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
    menu_handler.setCheckState(*link_wrapper.menu_joints,
        BoolToCheckState(has_joint_controls_));
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
        RAVELOG_DEBUG("Toggled enable to %d for '%s'\n",
            is_enabled, kinbody->GetName().c_str());
    }
    // Toggle kinbody visibility.
    else if (feedback->menu_entry_id == *link_wrapper.menu_visible) {
        MenuHandler::CheckState visible_state;
        menu_handler.getCheckState(*link_wrapper.menu_visible, visible_state);
        bool const is_visible = !CheckStateToBool(visible_state);
        kinbody->SetVisible(is_visible);
        RAVELOG_DEBUG("Toggled visible to %d for '%s'\n",
            is_visible, kinbody->GetName().c_str());
    }
    // Toggle full-KinBody joint controls.
    else if (feedback->menu_entry_id == *link_wrapper.menu_joints) {
        MenuHandler::CheckState joints_state;
        menu_handler.getCheckState(*link_wrapper.menu_joints, joints_state);
        has_joint_controls_ = !CheckStateToBool(joints_state);
        RAVELOG_DEBUG("Toggled joint controls to %d for '%s'\n",
            has_joint_controls_, kinbody->GetName().c_str());
    }
    // Toggle manipulator IK control.
    else if (link_wrapper.menu_manipulator_ik
             && feedback->menu_entry_id == link_wrapper.menu_manipulator_ik) {
        ManipulatorPtr const manipulator = link_wrapper.parent_manipulator;
        BOOST_ASSERT(manipulator);

        // TODO: Implement toggle functionality.
        bool const ik_enabled = true;

        if (ik_enabled) {
            ManipulatorMarkerPtr &manipulator_marker = manipulator_markers_[manipulator.get()];
            if (!manipulator_marker) {
                manipulator_marker = boost::make_shared<ManipulatorMarker>(server_, manipulator);
            }
        }

        RAVELOG_DEBUG("Toggled IK control to %d for '%s' manipulator '%s'.\n",
            ik_enabled, kinbody->GetName().c_str(),
            manipulator->GetName().c_str()
        );
    }

    UpdateMenu();
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
        bool const success = robot->GetChain(
                base_link->GetIndex(), tip_link->GetIndex(), chain_links);
        BOOST_ASSERT(success);

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
