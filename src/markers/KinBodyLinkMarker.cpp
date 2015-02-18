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
#include "markers/KinBodyLinkMarker.h"
#include "util/ros_conversions.h"

using interactive_markers::MenuHandler;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;

using namespace or_rviz::util;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::LinkInfo LinkInfo;

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

namespace or_rviz {
namespace markers {

KinBodyLinkMarker::KinBodyLinkMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                     OpenRAVE::KinBody::LinkPtr link)
    : LinkMarker(server, link, false)
{
    CreateMenu();
}

interactive_markers::MenuHandler &KinBodyLinkMarker::menu_handler()
{
    return menu_handler_;
}

bool KinBodyLinkMarker::EnvironmentSync()
{
    bool const is_changed = LinkMarker::EnvironmentSync();

    OpenRAVE::Transform const link_pose = link()->GetTransform();
    set_pose(link_pose);

    if (is_changed) {
        UpdateMenu();
    }

    return is_changed;
}

void KinBodyLinkMarker::CreateMenu()
{
    auto const callback = boost::bind(&KinBodyLinkMarker::MenuCallback, this, _1);

    menu_link_ = menu_handler_.insert("Link", callback);
    menu_enabled_ = menu_handler_.insert(menu_link_, "Enabled", callback);
    menu_visible_ = menu_handler_.insert(menu_link_, "Visible", callback);

    // Switching geometry mode.
    menu_geom_ = menu_handler_.insert(menu_link_, "Geometry");
    menu_geom_visual_ = menu_handler_.insert(menu_geom_, "Visual", callback);
    menu_geom_collision_ = menu_handler_.insert(menu_geom_, "Collision", callback);
    menu_geom_both_ = menu_handler_.insert(menu_geom_, "Both", callback);
    menu_changed_ = true;

    // Switching geometry groups.
    menu_groups_ = menu_handler_.insert(menu_link_, "Geometry Groups");
    menu_groups_entries_.clear();

    for (std::string const group_name : group_names()) {
        auto const callback = boost::bind(&KinBodyLinkMarker::SwitchGeometryGroup, this, group_name);
        menu_groups_entries_[group_name] = menu_handler_.insert(menu_groups_, group_name, callback);
    }
}

void KinBodyLinkMarker::UpdateMenu()
{
    LinkPtr const link = this->link();

    menu_handler_.setCheckState(menu_enabled_,
        BoolToCheckState(link->IsEnabled()));
    menu_handler_.setCheckState(menu_visible_,
        BoolToCheckState(link->IsVisible()));
    menu_handler_.setCheckState(menu_geom_visual_,
        BoolToCheckState(is_view_visual() && !is_view_collision()));
    menu_handler_.setCheckState(menu_geom_collision_,
        BoolToCheckState(!is_view_visual() && is_view_collision()));
    menu_handler_.setCheckState(menu_geom_both_,
        BoolToCheckState(is_view_visual() && is_view_collision()));

    menu_handler_.apply(*server_, interactive_marker_->name);
    menu_changed_ = false;
}

void KinBodyLinkMarker::MenuCallback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    LinkPtr const link = this->link();

    // Toggle collision detection.
    if (feedback->menu_entry_id == menu_enabled_) {
        MenuHandler::CheckState enabled_state;
        menu_handler_.getCheckState(menu_enabled_, enabled_state);

        bool const is_enabled = !CheckStateToBool(enabled_state);
        link->Enable(is_enabled);

        RAVELOG_DEBUG("Toggled enable to %d for '%s' link '%s'.\n",
            is_enabled, link->GetParent()->GetName().c_str(),
            link->GetName().c_str()
        );
    }
    // Toggle visiblity.
    else if (feedback->menu_entry_id == menu_visible_) {
        MenuHandler::CheckState visible_state;
        menu_handler_.getCheckState(menu_visible_, visible_state);

        bool const is_visible = !CheckStateToBool(visible_state);
        link->SetVisible(is_visible);

        RAVELOG_DEBUG("Toggled visible to %d for '%s' link '%s'.\n",
            is_visible, link->GetParent()->GetName().c_str(),
            link->GetName().c_str()
        );
    }
    // Geometry rendering mode.
    else if (feedback->menu_entry_id == menu_geom_visual_) {
        set_view_visual(true);
        set_view_collision(false);
        RAVELOG_DEBUG("Showing visual geometry for '%s' link '%s'.\n",
            link->GetParent()->GetName().c_str(),
            link->GetName().c_str()
        );
    }
    else if (feedback->menu_entry_id == menu_geom_collision_) {
        set_view_visual(false);
        set_view_collision(true);
        RAVELOG_DEBUG("Showing collision geometry for '%s' link '%s'.\n",
            link->GetParent()->GetName().c_str(),
            link->GetName().c_str()
        );
    }
    else if (feedback->menu_entry_id == menu_geom_both_) {
        set_view_visual(true);
        set_view_collision(true);
        RAVELOG_DEBUG("Showing visual and collision geometry for '%s' link '%s'.\n",
            link->GetParent()->GetName().c_str(),
            link->GetName().c_str()
        );
    }

    // TODO: Should we applyChanges here?
    UpdateMenu();
    server_->applyChanges();
}


}
}
