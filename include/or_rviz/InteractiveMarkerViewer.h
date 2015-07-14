/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#ifndef ORINTERACTIVEMARKER_H_
#define ORINTERACTIVEMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/signals2.hpp>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif
#include <interactive_markers/interactive_marker_server.h>
#include "markers/KinBodyMarker.h"
#include "util/InteractiveMarkerGraphHandle.h"

namespace or_rviz {

class InteractiveMarkerViewer : public OpenRAVE::ViewerBase {
public:
    InteractiveMarkerViewer(OpenRAVE::EnvironmentBasePtr env,
                            std::string const &topic_name);

    void set_environment(OpenRAVE::EnvironmentBasePtr const &env);
    void set_parent_frame(std::string const &frame_id);

    virtual void SetEnvironmentSync(bool do_update);
    virtual void EnvironmentSync();

    virtual int main(bool bShow = true);
    virtual void quitmainloop();

    virtual void Reset()
    {
        // Unimplemented in this viewer.
    }

    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr body)
    {
        // Unimplemented in this viewer.
    }

    virtual void SetCamera(const OpenRAVE::geometry::RaveTransform<float>& tf, float focalLength)
    {
        // Unimplemented in this viewer
    }

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

    virtual OpenRAVE::GraphHandlePtr drawarrow(
        OpenRAVE::RaveVector<float> const &p1,
        OpenRAVE::RaveVector<float> const &p2,
        float fwidth,
        OpenRAVE::RaveVector<float> const &color);

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

protected:
    typedef void ViewerCallbackFn();

    bool running_;
    bool do_sync_;
    std::string topic_name_;
    boost::signals2::signal<ViewerCallbackFn> viewer_callbacks_;

private:
    typedef bool SelectionCallbackFn(OpenRAVE::KinBody::LinkPtr plink,
                                     OpenRAVE::RaveVector<float>,
                                     OpenRAVE::RaveVector<float>);

    OpenRAVE::EnvironmentBasePtr env_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::UserDataPtr body_callback_handle_;
    boost::unordered_set<util::InteractiveMarkerGraphHandle *> graph_handles_;

    boost::signals2::signal<SelectionCallbackFn> selection_callbacks_;
    std::stringstream menu_queue_;

    bool parent_frame_id_changed_;
    std::string parent_frame_id_;

    // Arbitrarily convert openrave point pixel size to meters for rendering
    float pixels_to_meters_;

    bool AddMenuEntryCommand(std::ostream &out, std::istream &in);
    bool GetMenuSelectionCommand(std::ostream &out, std::istream &in);

    void GraphHandleRemovedCallback(util::InteractiveMarkerGraphHandle *handle);
    void BodyCallback(OpenRAVE::KinBodyPtr kinbody, int flag);
    void KinBodyMenuCallback(OpenRAVE::KinBodyPtr kinbody, std::string const &name);
    void LinkMenuCallback(OpenRAVE::KinBody::LinkPtr link, std::string const &name);
    void ManipulatorMenuCallback(OpenRAVE::RobotBase::ManipulatorPtr manipulator,
                                 std::string const &name);

    visualization_msgs::InteractiveMarkerPtr CreateMarker() const;
    util::InteractiveMarkerGraphHandlePtr CreateGraphHandle(
        visualization_msgs::InteractiveMarkerPtr const &marker
    );

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
