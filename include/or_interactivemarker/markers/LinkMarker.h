#ifndef LINKMARKER_H_
#define LINKMARKER_H_
#include <vector>
#include <boost/optional.hpp>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

namespace or_interactivemarker {
namespace markers {

class LinkMarker;
typedef boost::shared_ptr<LinkMarker> LinkMarkerPtr;

class LinkMarker {
public:
    static OpenRAVE::Vector const kCollisionColor;

    LinkMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
               OpenRAVE::KinBody::LinkPtr link, bool is_ghost);
    virtual ~LinkMarker();

    std::string id() const;
    OpenRAVE::KinBody::LinkPtr link() const;
    interactive_markers::MenuHandler &menu_handler();
    visualization_msgs::InteractiveMarkerPtr interactive_marker();

    void set_pose(OpenRAVE::Transform const &pose) const;

    void clear_color();
    void set_color(OpenRAVE::Vector const &color);

    bool is_view_visual() const;
    void set_view_visual(bool flag);

    bool is_view_collision() const;
    void set_view_collision(bool flag);

    void set_parent_frame(std::string const &frame_id);

    std::vector<std::string> group_names() const;
    void SwitchGeometryGroup(std::string const &group);

    virtual bool EnvironmentSync();
    void UpdateMenu();

protected:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    visualization_msgs::InteractiveMarkerPtr interactive_marker_;
    visualization_msgs::InteractiveMarkerControl *visual_control_;

private:
    OpenRAVE::KinBody::LinkWeakPtr link_;
    OpenRAVE::RobotBase::ManipulatorPtr manipulator_;
    bool is_ghost_;
    bool created_;
    bool force_update_;
    bool view_visual_;
    bool view_collision_;

    boost::optional<OpenRAVE::Vector> override_color_;

    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *, bool> visibility_map_;
    boost::unordered_map<
        OpenRAVE::KinBody::Link::Geometry *,
        std::vector<visualization_msgs::Marker *> > geometry_markers_;

    void CreateGeometry();
    visualization_msgs::MarkerPtr CreateVisualGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);
    visualization_msgs::MarkerPtr CreateCollisionGeometry(
            OpenRAVE::KinBody::Link::GeometryPtr geometry);

    bool HasTexture(std::string const &uri) const;
    bool HasRVizSupport(std::string const &uri) const;
    void TriMeshToMarker(OpenRAVE::TriMesh const &trimesh,
                         visualization_msgs::MarkerPtr const &marker);
};

}
}

#endif
