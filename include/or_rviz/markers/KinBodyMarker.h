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
#ifndef KINBODYMARKER_H_
#define KINBODYMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif
#include "KinBodyLinkMarker.h"
#include "KinBodyJointMarker.h"
#include "ManipulatorMarker.h"

namespace or_rviz {
namespace markers {

class KinBodyMarker;
typedef boost::shared_ptr<KinBodyMarker> KinBodyMarkerPtr;

struct LinkMarkerWrapper {
    typedef interactive_markers::MenuHandler::EntryHandle MenuEntry;

    LinkMarkerWrapper() : has_menu(false) { }

    KinBodyLinkMarkerPtr link_marker;
    OpenRAVE::RobotBase::ManipulatorPtr parent_manipulator;
    bool has_menu;
    boost::optional<MenuEntry> menu_parent;
    boost::optional<MenuEntry> menu_joints;
    boost::optional<MenuEntry> menu_enabled;
    boost::optional<MenuEntry> menu_visible;
    boost::optional<MenuEntry> menu_move;

    boost::optional<MenuEntry> menu_groups;
    boost::unordered_map<std::string, MenuEntry> menu_groups_entries;

    boost::optional<MenuEntry> menu_geometry;
    boost::optional<MenuEntry> menu_geometry_visual;
    boost::optional<MenuEntry> menu_geometry_collision;
    boost::optional<MenuEntry> menu_geometry_both;

    boost::optional<MenuEntry> menu_manipulator;
    boost::optional<MenuEntry> menu_manipulator_active;
    boost::optional<MenuEntry> menu_manipulator_joints;
    boost::optional<MenuEntry> menu_manipulator_ik;
};

struct CustomMenuEntry {
    std::string name;
    OpenRAVE::KinBody::LinkWeakPtr link;
    OpenRAVE::RobotBase::ManipulatorWeakPtr manipulator;
    boost::function<void ()> callback;
};

class KinBodyMarker : public OpenRAVE::UserData {
public:
    KinBodyMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                  OpenRAVE::KinBodyPtr kinbody);
    virtual ~KinBodyMarker();

    std::string id() const;

    void set_parent_frame(std::string const &frame_id);

    void AddMenuEntry(std::string const &name, boost::function<void ()> const &callback);
    void AddMenuEntry(OpenRAVE::KinBody::LinkPtr link,
                      std::string const &name, boost::function<void ()> const &callback);
    void AddMenuEntry(OpenRAVE::RobotBase::ManipulatorPtr manipulator,
                      std::string const &name, boost::function<void ()> const &callback);

    void EnvironmentSync();

    std::vector<std::string> group_names() const;
    void SwitchGeometryGroup(std::string const &group);

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBodyWeakPtr kinbody_;
    OpenRAVE::RobotBaseWeakPtr robot_;
    OpenRAVE::UserDataPtr handle_kinbody_;
    OpenRAVE::UserDataPtr handle_links_;
    OpenRAVE::UserDataPtr handle_manipulators_;
    std::string parent_frame_id_;
    bool has_pose_controls_;
    bool has_joint_controls_;

    visualization_msgs::InteractiveMarkerPtr interactive_marker_;

    std::vector<CustomMenuEntry> menu_custom_kinbody_;
    boost::unordered_map<OpenRAVE::KinBody::Link *, std::vector<CustomMenuEntry> > menu_custom_links_;
    boost::unordered_map<OpenRAVE::RobotBase::Manipulator *, std::vector<CustomMenuEntry> > menu_custom_manipulators_;

    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerWrapper> link_markers_;
    boost::unordered_map<OpenRAVE::KinBody::Joint *, KinBodyJointMarkerPtr> joint_markers_;
    boost::unordered_map<OpenRAVE::RobotBase::Manipulator *, ManipulatorMarkerPtr> manipulator_markers_;

    void InvalidateKinBody();
    void InvalidateLinks();
    void InvalidateManipulators();

    bool HasGhostManipulator(OpenRAVE::RobotBase::ManipulatorPtr const manipulator) const;

    void CreateMenu(LinkMarkerWrapper &link_wrapper);
    void UpdateMenu(LinkMarkerWrapper &link_wrapper);
    void UpdateMenu();
    void MenuCallback(LinkMarkerWrapper &link_wrapper,
                      visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    void CreatePoseControls();
    void EnablePoseControls(bool enabled);
    void PoseCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    void GetManipulators(OpenRAVE::KinBody::LinkPtr link,
                         std::vector<OpenRAVE::RobotBase::ManipulatorPtr> *manipulators) const;
};

}
}

#endif
