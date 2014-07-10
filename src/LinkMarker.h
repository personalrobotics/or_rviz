#ifndef LINKMARKER_H_
#define LINKMARKER_H_
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>

namespace or_interactivemarker {

class LinkMarker;
typedef boost::shared_ptr<LinkMarker> LinkMarkerPtr;

class LinkMarker {
public:
    LinkMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
               OpenRAVE::KinBody::LinkPtr link);

    visualization_msgs::InteractiveMarkerPtr interactive_marker() const;
    std::string id() const;

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBody::LinkPtr link_;
    visualization_msgs::InteractiveMarkerPtr interactive_marker_;
    visualization_msgs::InteractiveMarkerControl *visual_control_;
    bool changed_;
    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *,
        visualization_msgs::MarkerPtr> geometry_markers_;

    visualization_msgs::MarkerPtr CreateGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);
};

}

#endif
