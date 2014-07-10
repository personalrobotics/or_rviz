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
using interactive_markers::InteractiveMarkerServer;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

namespace or_interactivemarker {

LinkMarker::LinkMarker(boost::shared_ptr<InteractiveMarkerServer> server,
                       LinkPtr link)
    : link_(link)
    , server_(server)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(link);

    controls_.resize(1);
    visual_control_ = &controls_[0];
    visual_control_->orientation.w = 1;
    visual_control_->name = str(format("%s.Geometry[visual]") % id());
    visual_control_->orientation_mode = InteractiveMarkerControl::INHERIT;
    visual_control_->interaction_mode = InteractiveMarkerControl::NONE;
    visual_control_->always_visible = true;

    CreateControls();
    EnvironmentSync();
}

auto LinkMarker::begin() const -> const_iterator
{
    return controls_.begin();
}

auto LinkMarker::end() const -> const_iterator
{
    return controls_.end();
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
    OpenRAVE::Transform const body_pose = link_->GetParent()->GetTransform();
    OpenRAVE::Transform const link_pose = body_pose.inverse() * link_->GetTransform();

    for (GeometryPtr const geometry : link_->GetGeometries()) {
        auto const it = geometry_markers_.find(geometry.get());
        BOOST_ASSERT(it != geometry_markers_.end());
        Marker *marker = it->second;

        if (marker) {
            marker->pose = toROSPose(link_pose * geometry->GetTransform());
            marker->color = toROSColor(geometry->GetDiffuseColor());
            marker->color.a = 1.0; // TODO: Debug.
        }
    }
}

void LinkMarker::CreateControls()
{
    for (GeometryPtr const geometry : link_->GetGeometries()) {
        geometry_markers_[geometry.get()] = CreateGeometry(geometry);
    }
}

Marker *LinkMarker::CreateGeometry(GeometryPtr geometry)
{
    BOOST_ASSERT(geometry);

    // TODO: How should we allocate namespaces and IDs?
    Marker marker;
    marker.ns = str(format("%s.Geometry[%p]") % id() % geometry.get());
    marker.id = 0;

    if (!geometry->IsVisible()) {
        return NULL;
    }

    // If a render filename is specified, then we should ignore the rest of the
    // geometry. This is true regardless of the mesh type.
    std::string const render_mesh_path = geometry->GetRenderFilename();
    if (!render_mesh_path.empty()) {
        marker.type = Marker::MESH_RESOURCE;
        marker.scale = toROSVector(geometry->GetRenderScale());
        marker.mesh_resource = "file://" + render_mesh_path;
        marker.mesh_use_embedded_materials = true;

        visual_control_->markers.push_back(marker);
        return &visual_control_->markers.back();
    }

    // Otherwise, we have to render the underlying geometry type.
    switch (geometry->GetType()) {
    case OpenRAVE::GeometryType::GT_None:
        return NULL;

    case OpenRAVE::GeometryType::GT_Box:
        // TODO: This may be off by a factor of two.
        marker.type = Marker::CUBE;
        marker.scale = toROSVector(geometry->GetBoxExtents());
        break;

    case OpenRAVE::GeometryType::GT_Sphere: {
        double const sphere_radius = geometry->GetSphereRadius();
        marker.type = Marker::SPHERE;
        marker.scale.x = sphere_radius;
        marker.scale.y = sphere_radius;
        marker.scale.z = sphere_radius;
        break;
    }

    case OpenRAVE::GeometryType::GT_Cylinder: {
        // TODO: This may be rotated and/or off by a factor of two.
        double const cylinder_radius = geometry->GetCylinderRadius();
        double const cylinder_height= geometry->GetCylinderHeight();
        marker.type = Marker::CYLINDER;
        marker.scale.x = cylinder_radius;
        marker.scale.y = cylinder_radius;
        marker.scale.z = cylinder_height;
        break;
    }

    case OpenRAVE::GeometryType::GT_TriMesh:
        // TODO: Fall back on the OpenRAVE's mesh loader if this format is not
        // supported by RViz.
        return NULL;

    default:
        RAVELOG_WARN("Unknown geometry type '%d'.\n", geometry->GetType());
        return NULL;
    }

    visual_control_->markers.push_back(marker);
    return &visual_control_->markers.back();
}

}
