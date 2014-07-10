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
    typedef std::vector<visualization_msgs::InteractiveMarkerControl> ControlCollection;
    typedef ControlCollection::value_type value_type;
    typedef ControlCollection::iterator iterator;
    typedef ControlCollection::const_iterator const_iterator;

    LinkMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
               OpenRAVE::KinBody::LinkPtr link);

    std::string id() const;

    const_iterator begin() const;
    const_iterator end() const;

    void EnvironmentSync();

private:
    OpenRAVE::KinBody::LinkPtr link_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    std::vector<visualization_msgs::InteractiveMarkerControl> controls_;
    visualization_msgs::InteractiveMarkerControl *visual_control_;

    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *,
        visualization_msgs::Marker *> geometry_markers_;

    void CreateControls();
    visualization_msgs::Marker *CreateGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);
};

}

#endif
