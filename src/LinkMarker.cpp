#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>
#include <ros/ros.h>
#include "or_conversions.h"
#include "LinkMarker.h"

using boost::adaptors::transformed;
using boost::algorithm::join;
using boost::format;
using boost::str;
using geometry_msgs::Vector3;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::EnvironmentBasePtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerPtr;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using interactive_markers::MenuHandler;
using interactive_markers::InteractiveMarkerServer;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

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

OpenRAVE::Vector const LinkMarker::kGhostColor(0, 1, 0, 0.2);

LinkMarker::LinkMarker(boost::shared_ptr<InteractiveMarkerServer> server,
                       LinkPtr link, bool is_ghost)
    : server_(server)
    , link_(link)
    , is_ghost_(is_ghost)
    , interactive_marker_(boost::make_shared<InteractiveMarker>())
    , menu_changed_(false)
    , render_mode_(RenderMode::kVisual)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(link);

    // TODO: How should we handle this?
    //manipulator_ = InferManipulator();

    interactive_marker_->header.frame_id = kWorldFrameId;
    interactive_marker_->name = id();
    interactive_marker_->description = "";
    interactive_marker_->pose = toROSPose(link->GetTransform());
    interactive_marker_->scale = 0.25;

    // Show the visual geometry.
    interactive_marker_->controls.resize(1);
    visual_control_ = &interactive_marker_->controls[0];
    visual_control_->orientation.w = 1;
    visual_control_->name = str(format("%s.Geometry[visual]") % id());
    visual_control_->orientation_mode = InteractiveMarkerControl::INHERIT;
    visual_control_->interaction_mode = InteractiveMarkerControl::BUTTON;
    visual_control_->always_visible = true;

    // Create the right-click menu.
    CreateMenu();
}

LinkMarker::~LinkMarker()
{
    server_->erase(interactive_marker_->name);
}

InteractiveMarkerPtr LinkMarker::interactive_marker() const
{
    return interactive_marker_;
}

std::string LinkMarker::id() const
{
    LinkPtr link = link_.lock();
    KinBodyPtr const body = link->GetParent();
    EnvironmentBasePtr const env = body->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    return str(format("Environment[%d].KinBody[%s].Link[%s]")
               % environment_id % body->GetName() % link->GetName());
}

void LinkMarker::EnvironmentSync()
{
    LinkPtr link = link_.lock();
    bool is_changed = false;

    // Check if we need to re-create the marker to propagate changes in the
    // OpenRAVE environment.
    for (GeometryPtr const &geometry : link->GetGeometries()) {
        // Check if visibility changed.
        auto const it = geometry_markers_.find(geometry.get());
        bool const is_missing = it == geometry_markers_.end();
        bool const is_visible = geometry->IsVisible();
        is_changed = is_changed || (is_visible == is_missing);

        // TODO  Check if color changed.
        // TODO: Check if the transform changed.
        // TODO: Check if the geometry changed.
    }

    if (is_changed) {
        RAVELOG_DEBUG("Updating geometry for %s\n", id().c_str());
        CreateGeometry();
        server_->insert(*interactive_marker_);
    }
    // Incrementally update the marker's pose. We can't do this if we just
    // created the markers because the InteraciveMarkerServer will SEGFAULT.
    else {
        OpenRAVE::Transform const link_pose = link->GetTransform();
        server_->setPose(interactive_marker_->name, toROSPose(link_pose));
    }

    // Update the menu.
    if (menu_changed_) {
        UpdateMenu();
    }
}

void LinkMarker::CreateGeometry()
{
    visual_control_->markers.clear();
    geometry_markers_.clear();

    LinkPtr link = link_.lock();

    for (GeometryPtr const geometry : link->GetGeometries()) {
        MarkerPtr new_marker = CreateGeometry(geometry);
        if (new_marker) {
            visual_control_->markers.push_back(*new_marker);
            geometry_markers_[geometry.get()] = &visual_control_->markers.back();
        }
    }
}

void LinkMarker::CreateMenu()
{
    auto const callback = boost::bind(&LinkMarker::MenuCallback, this, _1);

    menu_link_ = menu_handler_.insert("Link", callback);
    menu_enabled_ = menu_handler_.insert(menu_link_, "Enabled", callback);
    menu_visible_ = menu_handler_.insert(menu_link_, "Visible", callback);
    menu_geom_visual_ = menu_handler_.insert(menu_link_, "Visual Geometry", callback);
    menu_geom_collision_ = menu_handler_.insert(menu_link_, "Collision Geometry", callback);
    menu_changed_ = true;
}

void LinkMarker::UpdateMenu()
{
    LinkPtr link = link_.lock();

    menu_handler_.setCheckState(menu_enabled_,
        BoolToCheckState(link->IsEnabled()));
    menu_handler_.setCheckState(menu_visible_,
        BoolToCheckState(link->IsVisible()));
    menu_handler_.setCheckState(menu_geom_visual_,
        BoolToCheckState(render_mode_ == RenderMode::kVisual));
    menu_handler_.setCheckState(menu_geom_collision_,
        BoolToCheckState(render_mode_ == RenderMode::kCollision));

    menu_handler_.apply(*server_, interactive_marker_->name);
    menu_changed_ = false;
}

void LinkMarker::MenuCallback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    LinkPtr link = link_.lock();

    // Toggle collision detection.
    {
        MenuHandler::CheckState enabled_state;
        menu_handler_.getCheckState(menu_enabled_, enabled_state);
        bool const is_enabled = CheckStateToBool(enabled_state);
        link->Enable(is_enabled);
    }

    // Toggle visiblity.
    {
        MenuHandler::CheckState visible_state;
        menu_handler_.getCheckState(menu_visible_, visible_state);
        bool const is_visible = !CheckStateToBool(visible_state);
        link->SetVisible(is_visible);
    }

    // Which type of geometry to render.
    {
        MenuHandler::CheckState visual_state, collision_state;
        menu_handler_.getCheckState(menu_geom_visual_, visual_state);
        menu_handler_.getCheckState(menu_geom_collision_, collision_state);

        if (visual_state == MenuHandler::CHECKED) {
            SetRenderMode(RenderMode::kVisual);
        } else if (collision_state == MenuHandler::CHECKED) {
            SetRenderMode(RenderMode::kCollision);
        } else {
            SetRenderMode(RenderMode::kNone);
        }
    }

#if 0
    UpdateMenu();
    server_->applyChanges();
#endif
}

void LinkMarker::SetRenderMode(RenderMode::Type mode)
{
    menu_changed_ = menu_changed_ || (mode != render_mode_);
    render_mode_ = mode;
}

MarkerPtr LinkMarker::CreateGeometry(GeometryPtr geometry)
{
    if (!geometry->IsVisible()) {
        return MarkerPtr();
    }

    MarkerPtr marker = boost::make_shared<Marker>();
    marker->pose = toROSPose(geometry->GetTransform());

    if (is_ghost_) {
        marker->color = toROSColor(kGhostColor);
    } else {
        marker->color = toROSColor(geometry->GetDiffuseColor());
        marker->color.a = 1.0 - geometry->GetTransparency();
    }

    // If a render filename is specified, then we should ignore the rest of the
    // geometry. This is true regardless of the mesh type.
    std::string render_mesh_path = geometry->GetRenderFilename();
    if (boost::algorithm::starts_with(render_mesh_path, "__norenderif__")) {
        render_mesh_path = "";
    }

    if (!render_mesh_path.empty()) {
        marker->type = Marker::MESH_RESOURCE;
        marker->scale = toROSVector(geometry->GetRenderScale());
        marker->mesh_resource = "file://" + render_mesh_path;
        marker->mesh_use_embedded_materials = !is_ghost_;
        return marker;
    }

    // Otherwise, we have to render the underlying geometry type.
    switch (geometry->GetType()) {
    case OpenRAVE::GeometryType::GT_None:
        return MarkerPtr();

    case OpenRAVE::GeometryType::GT_Box:
        // TODO: This may be off by a factor of two.
        marker->type = Marker::CUBE;
        marker->scale = toROSVector(geometry->GetBoxExtents());
        break;

    case OpenRAVE::GeometryType::GT_Sphere: {
        double const sphere_radius = geometry->GetSphereRadius();
        marker->type = Marker::SPHERE;
        marker->scale.x = sphere_radius;
        marker->scale.y = sphere_radius;
        marker->scale.z = sphere_radius;
        break;
    }

    case OpenRAVE::GeometryType::GT_Cylinder: {
        // TODO: This may be rotated and/or off by a factor of two.
        double const cylinder_radius = geometry->GetCylinderRadius();
        double const cylinder_height= geometry->GetCylinderHeight();
        marker->type = Marker::CYLINDER;
        marker->scale.x = cylinder_radius;
        marker->scale.y = cylinder_radius;
        marker->scale.z = cylinder_height;
        break;
    }

    case OpenRAVE::GeometryType::GT_TriMesh:
        // TODO: Fall back on the OpenRAVE's mesh loader if this format is not
        // supported by RViz.
        return MarkerPtr();
        break;

    default:
        RAVELOG_WARN("Unknown geometry type '%d'.\n", geometry->GetType());
        return MarkerPtr();
    }
    return marker;
}

ManipulatorPtr LinkMarker::InferManipulator()
{
    LinkPtr link = link_.lock();
    std::vector<ManipulatorPtr> manipulators;

    // TODO: What if this link is part of multiple manipulators?
    KinBodyPtr const kinbody = link->GetParent();
    RobotBasePtr const robot = boost::dynamic_pointer_cast<RobotBase>(kinbody);
    if (!robot) {
        return ManipulatorPtr();
    }

    for (ManipulatorPtr const manipulator : robot->GetManipulators()) {
        LinkPtr const base_link = manipulator->GetBase();

        // Check if this link is a child of the manipulator (i.e. gripper).
        std::vector<LinkPtr> child_links;
        manipulator->GetChildLinks(child_links);
        auto const it = std::find(child_links.begin(), child_links.end(), link);
        if (it != child_links.end()) {
            manipulators.push_back(manipulator);
            continue;
        }

        // Check if this link is in the manipulator chain by searching from
        // leaf to root.
        LinkPtr curr_link = manipulator->GetEndEffector();
        RAVELOG_INFO("Searching for parent '%s' of '%s'\n",
            curr_link->GetName().c_str(),
            base_link->GetName().c_str()
        );

        while (curr_link != base_link) {
            if (curr_link == link) {
                manipulators.push_back(manipulator);
                break;
            }

            std::vector<LinkPtr> parent_links;
            curr_link->GetParentLinks(parent_links);
            BOOST_ASSERT(parent_links.size() == 1);
            curr_link = parent_links.front();
        }
    }


    if (manipulators.empty()) {
        return ManipulatorPtr();
    } else if (manipulators.size() == 1) {
        return manipulators.front();
    } else {
        std::stringstream manipulator_names;
        for (ManipulatorPtr const manipulator : manipulators) {
            manipulator_names << " " << manipulator->GetName();
        }

        RAVELOG_WARN("Link '%s' is a member of %d manipulators [%s ]"
                     " [ %s ]. It will only be associated with manipulator %s"
                     " in the viewer.\n",
            link->GetName().c_str(),
            manipulators.size(),
            manipulator_names.str().c_str(),
            manipulators.front()->GetName().c_str()
        );
        return manipulators.front();
    }
}

}
