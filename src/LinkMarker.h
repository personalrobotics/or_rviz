#ifndef LINKMARKER_H_
#define LINKMARKER_H_
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace or_interactivemarker {

class LinkMarker;
typedef boost::shared_ptr<LinkMarker> LinkMarkerPtr;

class LinkMarker {
public:
    LinkMarker(OpenRAVE::KinBody::LinkPtr link);

    std::string GetId() const;
    void EnvironmentSync();

private:
    OpenRAVE::KinBody::LinkPtr link_;
    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *,
        visualization_msgs::InteractiveMarkerPtr> geometry_markers_;

    visualization_msgs::MarkerPtr CreateGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);
};

}

#endif
