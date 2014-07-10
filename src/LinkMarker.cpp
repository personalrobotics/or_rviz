#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include "or_conversions.h"
#include "LinkMarker.h"

using boost::format;
using boost::str;
using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerPtr;
typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;

namespace or_interactivemarker {

LinkMarker::LinkMarker(LinkPtr link)
    : link_(link)
{
    BOOST_ASSERT(link);
}

std::string LinkMarker::GetId() const
{
    OpenRAVE::KinBodyPtr const body = link_->GetParent();
    OpenRAVE::EnvironmentBasePtr const env = body->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    return str(format("Environment[%d].KinBody[%s].Link[%s]")
               % environment_id % body->GetName() % link_->GetName());
}

void LinkMarker::EnvironmentSync()
{
    std::vector<GeometryPtr> const &geometries = link_->GetGeometries();

    for (GeometryPtr geometry : geometries) {
        InteractiveMarkerPtr &interactive_marker = geometry_markers_[geometry.get()];
        if (!interactive_marker) {
            interactive_marker = boost::make_shared<InteractiveMarker>();
            //MarkerPtr marker = CreateGeometry(geometry);
        }

        interactive_marker->header.frame_id = "/world"; // TODO: Don't hardcode this.
        interactive_marker->name = GetId();
        //interactive_marker->pose
    }
}

MarkerPtr LinkMarker::CreateGeometry(GeometryPtr geometry)
{
    MarkerPtr marker = boost::make_shared<Marker>();
    marker->header.frame_id = "/world"; // TODO: Don't hardcode this.
    marker->action = Marker::ADD;
    marker->pose = toROSPose(geometry->GetTransform());
    marker->scale = toROSVector(geometry->GetRenderScale());
    marker->color = toROSColor(geometry->GetDiffuseColor());
    marker->ns = str(format("%s.Geometry[%p]") % GetId() % geometry.get());
    marker->id = 0;

    switch (geometry->GetType()) {
    case OpenRAVE::GeometryType::GT_None:
        return MarkerPtr();

    case OpenRAVE::GeometryType::GT_Box:
        marker->type = Marker::CUBE;
        break;

    case OpenRAVE::GeometryType::GT_Sphere:
        marker->type = Marker::SPHERE;
        break;

    case OpenRAVE::GeometryType::GT_Cylinder:
        marker->type = Marker::CYLINDER;
        break;

    case OpenRAVE::GeometryType::GT_TriMesh:
        marker->type = Marker::MESH_RESOURCE;
        marker->mesh_use_embedded_materials = true;
        marker->mesh_resource = geometry->GetRenderFilename();
        break;

    default:
        RAVELOG_WARN("Unknown geometry type '%d'.\n", geometry->GetType());
        return MarkerPtr();
    }
}

}
