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

    visualization_msgs::InteractiveMarkerPtr interactive_marker() const;
    std::string id() const;

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBody::LinkWeakPtr link_;
    OpenRAVE::RobotBase::ManipulatorPtr manipulator_;
    bool is_ghost_;

    std::vector<visualization_msgs::MenuEntry> menu_entries_;
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_entry_visual_;
    interactive_markers::MenuHandler::EntryHandle menu_entry_collision_;
    interactive_markers::MenuHandler::EntryHandle menu_entry_ik_;

    visualization_msgs::InteractiveMarkerPtr interactive_marker_;
    visualization_msgs::InteractiveMarkerControl *visual_control_;

    bool created_, menu_changed_;
    RenderMode::Type render_mode_;

    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *,
        visualization_msgs::Marker *> geometry_markers_;

    void CreateGeometry();

    void CreateMenu();
    void UpdateMenu();
    void MenuCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    void SetRenderMode(RenderMode::Type mode);
    OpenRAVE::RobotBase::ManipulatorPtr InferManipulator();

    visualization_msgs::MarkerPtr CreateGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);
};

}

#endif
