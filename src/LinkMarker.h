#ifndef LINKMARKER_H_
#define LINKMARKER_H_
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

namespace or_interactivemarker {

struct RenderMode {
    enum Type {
        kNone,
        kVisual,
        kCollision,
    };
};

class LinkMarker;
typedef boost::shared_ptr<LinkMarker> LinkMarkerPtr;

class LinkMarker {
public:
    static OpenRAVE::Vector const kGhostColor;

    LinkMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
               OpenRAVE::KinBody::LinkPtr link, bool is_ghost);
    virtual ~LinkMarker();

    std::string id() const;
    OpenRAVE::KinBody::LinkPtr link() const;
    void set_pose(OpenRAVE::Transform const &pose) const;
    interactive_markers::MenuHandler &menu_handler();
    visualization_msgs::InteractiveMarkerPtr interactive_marker();

    void SetRenderMode(RenderMode::Type mode);

    virtual bool EnvironmentSync();
    void UpdateMenu();

protected:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    visualization_msgs::InteractiveMarkerPtr interactive_marker_;
    visualization_msgs::InteractiveMarkerControl *visual_control_;

    RenderMode::Type render_mode_;

private:
    OpenRAVE::KinBody::LinkWeakPtr link_;
    OpenRAVE::RobotBase::ManipulatorPtr manipulator_;
    bool is_ghost_;
    bool created_;

    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *,
        visualization_msgs::Marker *> geometry_markers_;

    void CreateGeometry();
    visualization_msgs::MarkerPtr CreateGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);
};

}

#endif
