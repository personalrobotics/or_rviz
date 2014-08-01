#ifndef ORINTERACTIVEMARKER_H_
#define ORINTERACTIVEMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/signals2.hpp>
#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>
#include "KinBodyMarker.h"

namespace or_interactivemarker {

class InteractiveMarkerViewer : public OpenRAVE::ViewerBase {
public:
    InteractiveMarkerViewer(OpenRAVE::EnvironmentBasePtr env);

    virtual void EnvironmentSync();

    virtual int main(bool bShow = true);
    virtual void quitmainloop();

    virtual OpenRAVE::UserDataPtr RegisterItemSelectionCallback(
        OpenRAVE::ViewerBase::ItemSelectionCallbackFn const &fncallback);
    virtual OpenRAVE::UserDataPtr RegisterViewerThreadCallback(
        OpenRAVE::ViewerBase::ViewerThreadCallbackFn const &fncallback);

protected:
    virtual OpenRAVE::GraphHandlePtr plot3(
        float const *points, int num_points, int stride, float point_size,
        OpenRAVE::RaveVector<float> const &color, int draw_style = 0);
    virtual OpenRAVE::GraphHandlePtr plot3(
        float const *points, int num_points, int stride, float point_size,
        float const *colors, int draw_style = 0, bool has_alpha = false);

    virtual OpenRAVE::GraphHandlePtr drawlinestrip(
        float const *points, int num_points, int stride, float width,
        OpenRAVE::RaveVector<float> const &color);
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(
        float const *points, int num_points, int stride, float width,
        float const *colors);

    virtual OpenRAVE::GraphHandlePtr drawlinelist(
        float const *points, int num_points, int stride, float width,
        OpenRAVE::RaveVector<float> const &color);
    virtual OpenRAVE::GraphHandlePtr drawlinelist(
        float const *points, int num_points, int stride, float width,
        float const *colors);

    virtual OpenRAVE::GraphHandlePtr drawbox(
        OpenRAVE::RaveVector<float> const &position,
        OpenRAVE::RaveVector<float> const &extents);

    virtual OpenRAVE::GraphHandlePtr drawplane(
        OpenRAVE::RaveTransform<float> const &transform,
        OpenRAVE::RaveVector<float> const &extents,
        boost::multi_array<float, 3> const &texture);

    virtual OpenRAVE::GraphHandlePtr drawtrimesh(
        float const *points, int stride, int const *indices, int num_triangles,
        OpenRAVE::RaveVector<float> const &color);
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(
        float const *points, int stride, int const *pIndices, int num_triangles,
        boost::multi_array<float, 2> const &colors);

private:
    typedef void ViewerCallbackFn();
    typedef bool SelectionCallbackFn(OpenRAVE::KinBody::LinkPtr plink,
                                     OpenRAVE::RaveVector<float>,
                                     OpenRAVE::RaveVector<float>);

    OpenRAVE::EnvironmentBasePtr env_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    bool running_;
    int graph_id_;

    boost::signals2::signal<ViewerCallbackFn> viewer_callbacks_;
    boost::signals2::signal<SelectionCallbackFn> selection_callbacks_;
    std::stringstream menu_queue_;

    bool AddMenuEntryCommand(std::ostream &out, std::istream &in);
    bool GetMenuSelectionCommand(std::ostream &out, std::istream &in);

    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr body);

    void KinBodyMenuCallback(OpenRAVE::KinBodyPtr kinbody, std::string const &name);
    void LinkMenuCallback(OpenRAVE::KinBody::LinkPtr link, std::string const &name);
    void ManipulatorMenuCallback(OpenRAVE::RobotBase::ManipulatorPtr manipulator,
                                 std::string const &name);

    visualization_msgs::InteractiveMarkerPtr CreateMarker() const;
    void ConvertPoints(float const *points, int num_points, int stride,
                       std::vector<geometry_msgs::Point> *out_points) const;
    void ConvertColors(float const *colors, int num_colors, bool has_alpha,
                       std::vector<std_msgs::ColorRGBA> *out_colors) const;
    void ConvertMesh(float const *points, int stride,
                     int const *indices, int num_triangles,
                     std::vector<geometry_msgs::Point> *out_points) const;
};

}

#endif
