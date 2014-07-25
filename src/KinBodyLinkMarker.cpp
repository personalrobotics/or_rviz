#include <boost/range/adaptor/map.hpp>
#include "KinBodyLinkMarker.h"
#include "or_conversions.h"

using boost::adaptors::map_keys;
using interactive_markers::MenuHandler;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;

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

namespace or_interactivemarker {


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

    // Incrementally update the marker's pose. We can't do this if we just
    // created the markers because the InteraciveMarkerServer will SEGFAULT.
    if (!is_changed) {
        OpenRAVE::Transform const link_pose = link()->GetTransform();
        set_pose(link_pose);
    }

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

    LinkInfo const &link_info = link()->GetInfo();
    for (std::string const group_name : link_info._mapExtraGeometries | map_keys) {
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

