#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include "or_conversions.h"
#include "LinkMarker.h"

using boost::format;
using boost::str;
using geometry_msgs::Vector3;
using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerPtr;
using interactive_markers::InteractiveMarkerServer;
typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

// TODO: Don't hardcode this.
static std::string const world_frame_id = "/world";

namespace or_interactivemarker {

LinkMarker::LinkMarker(boost::shared_ptr<InteractiveMarkerServer> server,
                       LinkPtr link)
    : server_(server)
    , link_(link)
    , interactive_marker_(boost::make_shared<InteractiveMarker>())
    , changed_(false)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(link);

    interactive_marker_->header.frame_id = world_frame_id;
    interactive_marker_->name = id();
    interactive_marker_->description = "";
    interactive_marker_->pose = toROSPose(link_->GetTransform());
    interactive_marker_->scale = 1.0;
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
    for (GeometryPtr const geometry : link_->GetGeometries()) {
        MarkerPtr &marker = geometry_markers_[geometry.get()];
        if (!marker) {
            marker = CreateGeometry(geometry);
            // TODO: This shouldn't trigger if CreateGeometry does nothing.
            changed_ = true;
        }
    }
}

MarkerPtr LinkMarker::CreateGeometry(GeometryPtr geometry)
{
    MarkerPtr marker = boost::make_shared<Marker>();
    marker->header.frame_id = world_frame_id;
    marker->action = Marker::ADD;
    marker->pose = toROSPose(geometry->GetTransform());
    marker->color = toROSColor(geometry->GetDiffuseColor());

    // TODO: How should we allocate namespaces and IDs?
    marker->ns = str(format("%s.Geometry[%p]") % id() % geometry.get());
    marker->id = 0;

    if (geometry->IsVisible()) {
        return MarkerPtr();
    }

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
        marker->type = Marker::MESH_RESOURCE;
        marker->scale = toROSVector(geometry->GetRenderScale());
        marker->mesh_resource = geometry->GetRenderFilename();
        marker->mesh_use_embedded_materials = true;

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
