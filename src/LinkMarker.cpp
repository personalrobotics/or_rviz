#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include "or_conversions.h"
#include "LinkMarker.h"

using boost::format;
using boost::str;
using geometry_msgs::Vector3;
using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerPtr;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using interactive_markers::MenuHandler;
using interactive_markers::InteractiveMarkerServer;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

static MenuHandler::CheckState boolToCheckState(bool flag)
{
    if (flag) {
        return MenuHandler::CHECKED;
    } else {
        return MenuHandler::UNCHECKED;
    }
}

namespace or_interactivemarker {

LinkMarker::LinkMarker(boost::shared_ptr<InteractiveMarkerServer> server,
                       LinkPtr link)
    : server_(server)
    , link_(link)
    , interactive_marker_(boost::make_shared<InteractiveMarker>())
    , created_(false)
    , menu_changed_(true)
    , render_mode_(RenderMode::kVisual)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(link);

    interactive_marker_->header.frame_id = kWorldFrameId;
    interactive_marker_->name = id();
    interactive_marker_->description = "";
    interactive_marker_->pose = toROSPose(link_->GetTransform());
    interactive_marker_->scale = 0.25;

    // Show the visual geometry.
    interactive_marker_->controls.resize(1);
    visual_control_ = &interactive_marker_->controls[0];
    visual_control_->orientation.w = 1;
    visual_control_->name = str(format("%s.Geometry[visual]") % id());
    visual_control_->orientation_mode = InteractiveMarkerControl::INHERIT;
    visual_control_->interaction_mode = InteractiveMarkerControl::BUTTON;
    visual_control_->always_visible = true;
    CreateGeometry();

    // Create the right-click menu.
    CreateMenu();

    server_->insert(*interactive_marker_);
}

InteractiveMarkerPtr LinkMarker::interactive_marker() const
{
    return interactive_marker_;
}

std::string LinkMarker::id() const
{
    OpenRAVE::KinBodyPtr const body = link_->GetParent();
    OpenRAVE::EnvironmentBasePtr const env = body->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    return str(format("Environment[%d].KinBody[%s].Link[%s]")
               % environment_id % body->GetName() % link_->GetName());
}

void LinkMarker::EnvironmentSync()
{
    // TODO: Update other properties (e.g. IsVisible).
    // TODO: Update the geometry properties.

    if (created_) {
        OpenRAVE::Transform const link_pose = link_->GetTransform();
        server_->setPose(interactive_marker_->name, toROSPose(link_pose));
    }
    created_ = true;

    // Update the menu.
    if (menu_changed_) {
        UpdateMenu();
    }
}

void LinkMarker::CreateGeometry()
{
    for (GeometryPtr const geometry : link_->GetGeometries()) {
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
    menu_entry_visual_ = menu_handler_.insert("Visual Geometry", callback);
    menu_entry_collision_ = menu_handler_.insert("Collision Geometry", callback);
}

void LinkMarker::UpdateMenu()
{
    menu_handler_.setCheckState(menu_entry_visual_,
        boolToCheckState(render_mode_ == RenderMode::kVisual));
    menu_handler_.setCheckState(menu_entry_collision_,
        boolToCheckState(render_mode_ == RenderMode::kCollision));

    menu_handler_.apply(*server_, interactive_marker_->name);
    menu_changed_ = false;
}

void LinkMarker::MenuCallback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    MenuHandler::CheckState visual_state, collision_state;
    menu_handler_.getCheckState(menu_entry_visual_, visual_state);
    menu_handler_.getCheckState(menu_entry_collision_, collision_state);

    if (visual_state == MenuHandler::CHECKED) {
        SetRenderMode(RenderMode::kVisual);
    } else if (collision_state == MenuHandler::CHECKED) {
        SetRenderMode(RenderMode::kCollision);
    } else {
        SetRenderMode(RenderMode::kNone);
    }

    UpdateMenu();
}

void LinkMarker::SetRenderMode(RenderMode::Type mode)
{
    menu_changed_ = menu_changed_ || (mode != render_mode_);
    render_mode_ = mode;
}

MarkerPtr LinkMarker::CreateGeometry(GeometryPtr geometry)
{
    MarkerPtr marker = boost::make_shared<Marker>();
    marker->pose = toROSPose(geometry->GetTransform());
    marker->color = toROSColor(geometry->GetDiffuseColor());
    marker->color.a = 1.0; // TODO: Debug.

    // TODO: How should we allocate namespaces and IDs?
    marker->ns = str(format("%s.Geometry[%p]") % id() % geometry.get());
    marker->id = 0;

    if (!geometry->IsVisible()) {
        return MarkerPtr();
    }

    // If a render filename is specified, then we should ignore the rest of the
    // geometry. This is true regardless of the mesh type.
    std::string const render_mesh_path = geometry->GetRenderFilename();
    if (!render_mesh_path.empty()) {
        marker->type = Marker::MESH_RESOURCE;
        marker->scale = toROSVector(geometry->GetRenderScale());
        marker->mesh_resource = "file://" + render_mesh_path;
        marker->mesh_use_embedded_materials = true;
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
        break;

    default:
        RAVELOG_WARN("Unknown geometry type '%d'.\n", geometry->GetType());
        return MarkerPtr();
    }
    return marker;
}

}
