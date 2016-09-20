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
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include "util/ScopedConnection.h"
#include "util/ros_conversions.h"
#include "InteractiveMarkerViewer.h"

using boost::format;
using boost::str;
using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerPtr;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::GraphHandlePtr;

using namespace or_rviz::markers;
using namespace or_rviz::util;

typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

static double const kRefreshRate = 30;
static double const kWidthScaleFactor = 100;

namespace or_rviz {

namespace {

std::string GetRemainingContent(std::istream &stream, bool trim = false)
{

    std::istreambuf_iterator<char> eos;
    std::string str(std::istreambuf_iterator<char>(stream), eos);
    if (trim) {
        boost::algorithm::trim(str);
    }
    return str;
}

}

InteractiveMarkerViewer::InteractiveMarkerViewer(
        OpenRAVE::EnvironmentBasePtr env,
        std::string const &topic_name)
    : OpenRAVE::ViewerBase(env)
    , running_(false)
    , do_sync_(true)
    , topic_name_(topic_name)
    , server_(boost::make_shared<InteractiveMarkerServer>(topic_name))
    , parent_frame_id_(kDefaultWorldFrameId)
    , pixels_to_meters_(0.001)
{
    BOOST_ASSERT(env);

    RegisterCommand("AddMenuEntry",
        boost::bind(&InteractiveMarkerViewer::AddMenuEntryCommand, this, _1, _2),
        "Attach a custom menu entry to an object."
    );
    RegisterCommand("GetMenuSelection",
        boost::bind(&InteractiveMarkerViewer::GetMenuSelectionCommand, this, _1, _2),
        "Get the name of the last menu selection."
    );

    set_environment(env);
}

void InteractiveMarkerViewer::set_environment(
    OpenRAVE::EnvironmentBasePtr const &env)
{
    BOOST_ASSERT(env);

    RAVELOG_DEBUG("Switching to environment %d.\n",
        OpenRAVE::RaveGetEnvironmentId(env));

    // Register a callback to listen for bodies being added and remove.
    body_callback_handle_ = env->RegisterBodyCallback(
        boost::bind(&InteractiveMarkerViewer::BodyCallback, this, _1, _2)
    );

    // Manually remove all bodies in the old environment.
    if (env_) {
        std::vector<OpenRAVE::KinBodyPtr> old_bodies;
        env_->GetBodies(old_bodies);

        for (OpenRAVE::KinBodyPtr const &body : old_bodies) {
            BodyCallback(body, 0);
        }
    }

    // Switch to the new environment.
    env_ = env;

    // Manually insert all bodies in the new environment.
    std::vector<OpenRAVE::KinBodyPtr> new_bodies;
    env_->GetBodies(new_bodies);

    for (OpenRAVE::KinBodyPtr const &body : new_bodies) {
        BodyCallback(body, 1);
    }
}

void InteractiveMarkerViewer::set_parent_frame(std::string const &frame_id)
{
    if (frame_id != parent_frame_id_) {
        RAVELOG_DEBUG("Changed parent frame ID from '%s' to '%s'.\n",
            parent_frame_id_.c_str(), frame_id.c_str());
    }

    parent_frame_id_ = frame_id;

    // TODO: Also re-create any visualization markers in the correct frame.
}

int InteractiveMarkerViewer::main(bool bShow)
{
    ros::Rate rate(kRefreshRate);

    RAVELOG_DEBUG("Starting main loop with a %.0f Hz refresh rate.\n",
        kRefreshRate
    );

    running_ = true;
    while (running_) {
        if (do_sync_) {
            EnvironmentSync();
        }
        viewer_callbacks_();
        rate.sleep();
    }

    RAVELOG_DEBUG("Exiting main loop.\n");
    return 0;
}

void InteractiveMarkerViewer::quitmainloop()
{
    RAVELOG_DEBUG("Stopping main loop on the cycle (within %.3f ms).\n",
        1.0 / kRefreshRate
    );
    running_ = false;
}

void InteractiveMarkerViewer::EnvironmentSync()
{
    // TODO: Do I need to lock here? Is the environment already locked?
    OpenRAVE::EnvironmentMutex::scoped_lock lock(env_->GetMutex(),
                                                 boost::try_to_lock);
    if (!lock) {
        return;
    }

    std::vector<KinBodyPtr> bodies;
    env_->GetBodies(bodies);

    for (KinBodyPtr const &body : bodies) {
        OpenRAVE::UserDataPtr raw = body->GetUserData("interactive_marker"); 
        auto body_marker = boost::dynamic_pointer_cast<KinBodyMarker>(raw);

        // It's possibe to get here without the body's KinBodyMarker being
        // fully initialized due to a race condition and/or environment
        // cloning, which doesn't call BodyCallback.
        if (!body_marker) {
            BodyCallback(body, 1);

            raw = body->GetUserData("interactive_marker"); 
            body_marker = boost::dynamic_pointer_cast<KinBodyMarker>(raw);
            BOOST_ASSERT(body_marker);
        }

        body_marker->set_parent_frame(parent_frame_id_);
        body_marker->EnvironmentSync();
    }

    // Update any graph handles.
    for (util::InteractiveMarkerGraphHandle *const handle : graph_handles_) {
        handle->set_parent_frame(parent_frame_id_);
    }

    server_->applyChanges();
    ros::spinOnce();
}

void InteractiveMarkerViewer::SetEnvironmentSync(bool do_update)
{
    do_sync_ = do_update;
}

OpenRAVE::UserDataPtr InteractiveMarkerViewer::RegisterItemSelectionCallback(
    OpenRAVE::ViewerBase::ItemSelectionCallbackFn const &fncallback)
{
    boost::signals2::connection const con = selection_callbacks_.connect(fncallback);
    return boost::make_shared<util::ScopedConnection>(con);
    return OpenRAVE::UserDataPtr();
}

OpenRAVE::UserDataPtr InteractiveMarkerViewer::RegisterViewerThreadCallback(
    OpenRAVE::ViewerBase::ViewerThreadCallbackFn const &fncallback)
{
    boost::signals2::connection const con = viewer_callbacks_.connect(fncallback);
    return boost::make_shared<util::ScopedConnection>(con);
}

GraphHandlePtr InteractiveMarkerViewer::plot3(
    float const *points, int num_points, int stride, float point_size,
    OpenRAVE::RaveVector<float> const &color, int draw_style)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.color = toROSColor<>(color);

    if (draw_style == 0) {
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = point_size * pixels_to_meters_;
        marker.scale.y = point_size * pixels_to_meters_;
    } else if (draw_style == 1) {
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.scale.x = point_size * pixels_to_meters_;
        marker.scale.y = point_size * pixels_to_meters_;
        marker.scale.z = point_size * pixels_to_meters_;
    } else {
        throw OpenRAVE::openrave_exception(str(
            format("Unsupported drawstyle %d; expected 0 or 1.")
                % draw_style
            ), OpenRAVE::ORE_InvalidArguments
        );
    }

    ConvertPoints(points, num_points, stride, &marker.points);

    return CreateGraphHandle(interactive_marker);
}

OpenRAVE::GraphHandlePtr InteractiveMarkerViewer::plot3(
    float const *points, int num_points, int stride, float point_size,
    float const *colors, int draw_style, bool has_alpha)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();

    if (draw_style == 0) {
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = point_size * pixels_to_meters_;
        marker.scale.y = point_size * pixels_to_meters_;
    } else if (draw_style == 1) {
        // TODO: Does this support individual colors?
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.scale.x = point_size * pixels_to_meters_;
        marker.scale.y = point_size * pixels_to_meters_;
        marker.scale.z = point_size * pixels_to_meters_;
    } else {
        throw OpenRAVE::openrave_exception(str(
            format("Unsupported drawstyle %d; expected 0 or 1.")
                % draw_style
            ), OpenRAVE::ORE_InvalidArguments
        );
    }

    ConvertPoints(points, num_points, stride, &marker.points);
    ConvertColors(colors, num_points, has_alpha, &marker.colors);

    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawarrow(
    OpenRAVE::RaveVector<float> const &p1,
    OpenRAVE::RaveVector<float> const &p2,
    float fwidth,
    OpenRAVE::RaveVector<float> const &color)
{
    visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.color = toROSColor<>(color);
    marker.scale.x = fwidth * 1.0f;
    marker.scale.y = fwidth * 1.5f;
    marker.scale.z = fwidth * 2.0f;
    marker.points.push_back(toROSPoint(p1));
    marker.points.push_back(toROSPoint(p2));


    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawlinestrip(
    float const *points, int num_points, int stride, float width,
    OpenRAVE::RaveVector<float> const &color)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color = toROSColor<>(color);
    marker.scale.x = width * pixels_to_meters_;

    ConvertPoints(points, num_points, stride, &marker.points);

    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawlinestrip(
    float const *points, int num_points, int stride, float width,
    float const *colors)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = width * pixels_to_meters_;

    ConvertPoints(points, num_points, stride, &marker.points);
    ConvertColors(colors, num_points, false, &marker.colors);

    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawlinelist(
    float const *points, int num_points, int stride, float width,
    OpenRAVE::RaveVector<float> const &color)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.color = toROSColor<>(color);
    marker.scale.x = width / kWidthScaleFactor;

    ConvertPoints(points, num_points, stride, &marker.points);

    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawlinelist(
    float const *points, int num_points, int stride, float width,
    float const *colors)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = width / kWidthScaleFactor;

    ConvertPoints(points, num_points, stride, &marker.points);
    ConvertColors(colors, num_points, false, &marker.colors);

    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawbox(
    OpenRAVE::RaveVector<float> const &pos,
    OpenRAVE::RaveVector<float> const &extents)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position = toROSPoint<>(pos);
    marker.scale = toROSVector<>(2.0 * extents);

    return CreateGraphHandle(interactive_marker);
}

OpenRAVE::GraphHandlePtr InteractiveMarkerViewer::drawplane(
    OpenRAVE::RaveTransform<float> const &transform,
    OpenRAVE::RaveVector<float> const &extents,
    boost::multi_array<float, 3> const &texture)
{
    throw OpenRAVE::openrave_exception(
        "drawplane is not implemented on InteractiveMarkerViewer",
        OpenRAVE::ORE_NotImplemented
    );
}

GraphHandlePtr InteractiveMarkerViewer::drawtrimesh(
    float const *points, int stride, int const *indices, int num_triangles,
    OpenRAVE::RaveVector<float> const &color)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.color = toROSColor<>(color);
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    ConvertMesh(points, stride, indices, num_triangles, &marker.points);

    return CreateGraphHandle(interactive_marker);
}

GraphHandlePtr InteractiveMarkerViewer::drawtrimesh(
    float const *points, int stride, int const *indices, int num_triangles,
    boost::multi_array<float, 2> const &colors)
{
    InteractiveMarkerPtr interactive_marker = CreateMarker();
    visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;


    ConvertMesh(points, stride, indices, num_triangles, &marker.points);

    // TODO: Colors should be per-vertex, not per-face.
    size_t const *color_shape = colors.shape();
    if (color_shape[0] != num_triangles) {
        throw OpenRAVE::openrave_exception(str(
            format("Number of colors does not equal number of triangles;"
                   " expected %d, got %d.")
                % color_shape[0] % num_triangles
            ),
            OpenRAVE::ORE_InvalidArguments
        );
    } else if (color_shape[1] != 3 && color_shape[1] != 4) {
        throw OpenRAVE::openrave_exception(str(
            format("Invalid number of channels; expected 3 or 4, got %d.")
                % color_shape[1]
            ),
            OpenRAVE::ORE_InvalidArguments
        );
    }

    marker.colors.resize(3 * num_triangles);
    for (int itri = 0; itri < num_triangles; ++itri) {
        std_msgs::ColorRGBA color;
        color.r = colors[itri][0];
        color.g = colors[itri][1];
        color.b = colors[itri][2];

        if (color_shape[1] == 4) {
            color.a = colors[itri][3];
        } else {
            color.a = 1.0;
        }

        for (int ivertex = 0; ivertex < 3; ++ivertex) {
            int const index_offset = 3 * itri + ivertex;
            int const index = stride * indices[index_offset];
            marker.colors[index] = color;
        }
    }

    return CreateGraphHandle(interactive_marker);
}

bool InteractiveMarkerViewer::AddMenuEntryCommand(std::ostream &out,
                                                  std::istream &in)
{
    std::string type, kinbody_name;
    in >> type >> kinbody_name;

    // Get the KinBodyMarker associated with the target object.
    OpenRAVE::KinBodyPtr const kinbody = env_->GetKinBody(kinbody_name);
    if (!kinbody) {
        throw OpenRAVE::openrave_exception(
            str(format("There is no KinBody named '%s' in the environment.")
                % kinbody_name),
            OpenRAVE::ORE_Failed
        );
    }

    OpenRAVE::UserDataPtr const marker_raw = kinbody->GetUserData("interactive_marker"); 
    auto const marker = boost::dynamic_pointer_cast<KinBodyMarker>(marker_raw);
    if (!marker) {
        throw OpenRAVE::openrave_exception(
            str(format("KinBody '%s' does not have an associated marker.")
                %  kinbody_name),
            OpenRAVE::ORE_InvalidState
        );
    }

    if (type == "kinbody") {
        std::string const name = GetRemainingContent(in, true);
        auto const callback = boost::bind(
            &InteractiveMarkerViewer::KinBodyMenuCallback,
            this, kinbody, name
        );
        marker->AddMenuEntry(name, callback);
    } else if (type == "link") {
        std::string link_name;
        in >> link_name;

        OpenRAVE::KinBody::LinkPtr const link = kinbody->GetLink(link_name);
        if (!link) {
            throw OpenRAVE::openrave_exception(
                str(format("KinBody '%s' has no link '%s'.")
                    % kinbody_name % link_name),
                OpenRAVE::ORE_Failed
            );
        }

        std::string const name = GetRemainingContent(in, true);
        auto const callback = boost::bind(
            &InteractiveMarkerViewer::LinkMenuCallback,
            this, link, name
        );
        marker->AddMenuEntry(link, name, callback);
    } else if (type == "manipulator" || type == "ghost_manipulator") {
        std::string manipulator_name;
        in >> manipulator_name;

        if (!kinbody->IsRobot()) {
            throw OpenRAVE::openrave_exception(
                str(format("KinBody '%s' is not a robot and does not support"
                           " manipulator menus.")
                    % kinbody_name),
                OpenRAVE::ORE_Failed
            );
        }

        auto const robot = boost::dynamic_pointer_cast<OpenRAVE::RobotBase>(kinbody);
        OpenRAVE::RobotBase::ManipulatorPtr const manipulator = robot->GetManipulator(manipulator_name);
        if (!manipulator) {
            throw OpenRAVE::openrave_exception(
                str(format("Robot '%s' has no manipulator '%s'.")
                    % kinbody_name % manipulator_name),
                OpenRAVE::ORE_Failed
            );
        }

        std::string const name = GetRemainingContent(in, true);
        auto const callback = boost::bind(
            &InteractiveMarkerViewer::ManipulatorMenuCallback,
            this, manipulator, name
        );
        marker->AddMenuEntry(manipulator, name, callback);
    }
    return true;
}

void InteractiveMarkerViewer::GraphHandleRemovedCallback(
        util::InteractiveMarkerGraphHandle *handle)
{
    graph_handles_.erase(handle);
}

bool InteractiveMarkerViewer::GetMenuSelectionCommand(std::ostream &out,
                                                      std::istream &in)
{
    out << menu_queue_.rdbuf();
}

void InteractiveMarkerViewer::BodyCallback(OpenRAVE::KinBodyPtr body, int flag)
{
    RAVELOG_DEBUG("BodyCallback %s -> %d\n", body->GetName().c_str(), flag);

    // Added.
    if (flag == 1) {
        auto const body_marker = boost::make_shared<KinBodyMarker>(server_, body);
        body_marker->set_parent_frame(parent_frame_id_);
        body->SetUserData("interactive_marker", body_marker);
    }
    // Removed.
    else if (flag == 0) {
        body->RemoveUserData("interactive_marker");
    }
}

void InteractiveMarkerViewer::KinBodyMenuCallback(OpenRAVE::KinBodyPtr kinbody,
                                                  std::string const &name)
{
    menu_queue_ << "kinbody " << kinbody->GetName()
                << " " << name << '\n';
    selection_callbacks_(OpenRAVE::KinBody::LinkPtr(),
                         OpenRAVE::RaveVector<float>(),
                         OpenRAVE::RaveVector<float>());
}

void InteractiveMarkerViewer::LinkMenuCallback(OpenRAVE::KinBody::LinkPtr link,
                                               std::string const &name)
{
    menu_queue_ << "link " << link->GetParent()->GetName()
                << " " << link->GetName()
                << " " << name << '\n';
}

void InteractiveMarkerViewer::ManipulatorMenuCallback(
        OpenRAVE::RobotBase::ManipulatorPtr manipulator, std::string const &name)
{
    menu_queue_ << "manipulator " << manipulator->GetRobot()->GetName()
                << " " << manipulator->GetName()
                << " " << name << '\n';
}

util::InteractiveMarkerGraphHandlePtr InteractiveMarkerViewer::CreateGraphHandle(
    visualization_msgs::InteractiveMarkerPtr const &interactive_marker)
{
    auto const handle = boost::make_shared<util::InteractiveMarkerGraphHandle>(
        server_, interactive_marker,
        boost::bind(&InteractiveMarkerViewer::GraphHandleRemovedCallback,
                    this, _1)
    );
    graph_handles_.insert(handle.get());
    return handle;
}

InteractiveMarkerPtr InteractiveMarkerViewer::CreateMarker() const
{
    auto interactive_marker = boost::make_shared<InteractiveMarker>();

    interactive_marker->header.frame_id = parent_frame_id_;
    interactive_marker->pose = toROSPose(OpenRAVE::Transform());
    interactive_marker->name = str(format("GraphHandle[%p]") % interactive_marker.get());
    interactive_marker->scale = 1.0;

    interactive_marker->controls.resize(1);
    visualization_msgs::InteractiveMarkerControl &control = interactive_marker->controls.front();
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    control.always_visible = true;

    control.markers.resize(1);
    visualization_msgs::Marker &marker = control.markers.front();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    return interactive_marker;
}

void InteractiveMarkerViewer::ConvertPoints(
        float const *points, int num_points, int stride,
        std::vector<geometry_msgs::Point> *out_points) const
{
    BOOST_ASSERT(points);
    BOOST_ASSERT(num_points >= 0);
    BOOST_ASSERT(stride >= 0 && stride % sizeof(float) == 0);
    BOOST_ASSERT(out_points);

    stride = stride / sizeof(float);

    out_points->resize(num_points);
    for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
        geometry_msgs::Point &out_point = out_points->at(ipoint);
        out_point.x = points[stride * ipoint + 0];
        out_point.y = points[stride * ipoint + 1];
        out_point.z = points[stride * ipoint + 2];
    }
}

void InteractiveMarkerViewer::ConvertColors(
        float const *colors, int num_colors, bool has_alpha,
        std::vector<std_msgs::ColorRGBA> *out_colors) const
{
    BOOST_ASSERT(colors);
    BOOST_ASSERT(num_colors >= 0);
    BOOST_ASSERT(out_colors);

    int stride;
    if (has_alpha) {
        stride = 4;
    } else {
        stride = 3;
    }

    out_colors->resize(num_colors);
    for (size_t icolor = 0; icolor < num_colors; ++icolor) {
        std_msgs::ColorRGBA &out_color = out_colors->at(icolor);
        out_color.r = colors[icolor * stride + 0];
        out_color.g = colors[icolor * stride + 1];
        out_color.b = colors[icolor * stride + 2];

        if (has_alpha) {
            out_color.a = colors[icolor * stride + 3];
        } else {
            out_color.a = 1.0;
        }
    }
}

void InteractiveMarkerViewer::ConvertMesh(
        float const *points, int stride, int const *indices, int num_triangles,
        std::vector<geometry_msgs::Point> *out_points) const
{
    BOOST_ASSERT(points);
    BOOST_ASSERT(stride > 0);
    BOOST_ASSERT(indices);
    BOOST_ASSERT(num_triangles >= 0);
    BOOST_ASSERT(out_points);

    auto const points_raw = reinterpret_cast<uint8_t const *>(points);

    if (num_triangles == 0) {
        out_points->resize(3);
        for (int ivertex = 0; ivertex < 3; ++ivertex) {
            geometry_msgs::Point &point = (*out_points)[ivertex];
            point.x = 0.;
            point.y = 0.;
            point.z = 0.;
        }
    } else {
        out_points->resize(3 * num_triangles);
        for (int iindex = 0; iindex < num_triangles; ++iindex) {
            for (int ivertex = 0; ivertex < 3; ++ivertex) {
                int const index_offset = 3 * iindex + ivertex;
                float const *or_point = reinterpret_cast<float const *>(
                    points_raw + stride * indices[index_offset]
                );

                geometry_msgs::Point &out_point = out_points->at(3 * iindex + ivertex);
                out_point.x = or_point[0];
                out_point.y = or_point[1];
                out_point.z = or_point[2];
            }
        }
    }
}

}
