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
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/adaptor/map.hpp>
#include <ros/ros.h>
#include "markers/LinkMarker.h"
#include "util/ros_conversions.h"

using boost::adaptors::map_keys;
using boost::adaptors::transformed;
using boost::algorithm::join;
using boost::algorithm::iends_with;
using boost::format;
using boost::str;
using geometry_msgs::Vector3;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::EnvironmentBasePtr;
using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerPtr;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using interactive_markers::InteractiveMarkerServer;

using namespace or_rviz::util;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef boost::shared_ptr<OpenRAVE::TriMesh> TriMeshPtr;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

namespace or_rviz {
namespace markers {

OpenRAVE::Vector const LinkMarker::kCollisionColor(0.0, 0.0, 1.0, 0.5);

LinkMarker::LinkMarker(boost::shared_ptr<InteractiveMarkerServer> server,
                       LinkPtr link, bool is_ghost)
    : server_(server)
    , interactive_marker_(boost::make_shared<InteractiveMarker>())
    , view_visual_(true)
    , view_collision_(false)
    , link_(link)
    , is_ghost_(is_ghost)
    , force_update_(true)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(link);

    // TODO: How should we handle this?
    //manipulator_ = InferManipulator();

    interactive_marker_->header.frame_id = kDefaultWorldFrameId;
    interactive_marker_->name = id();
    interactive_marker_->description = "";
    interactive_marker_->pose = toROSPose(link->GetTransform());
    interactive_marker_->scale = 0.25;

    // Show the visual geometry.
    interactive_marker_->controls.resize(1);
    visual_control_ = &interactive_marker_->controls[0];
    visual_control_->orientation.w = 1;
    visual_control_->name = str(format("%s.Geometry[visual]") % id());
    visual_control_->orientation_mode = InteractiveMarkerControl::INHERIT;
    visual_control_->interaction_mode = InteractiveMarkerControl::BUTTON;
    visual_control_->always_visible = true;
}

LinkMarker::~LinkMarker()
{
    server_->erase(interactive_marker_->name);
}

std::string LinkMarker::id() const
{
    LinkPtr const link = this->link();
    KinBodyPtr const body = link->GetParent();
    EnvironmentBasePtr const env = body->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    std::string suffix;
    if (is_ghost_) {
        suffix = ".Ghost";
    }

    return str(format("Environment[%d].KinBody[%s].Link[%s]%s")
               % environment_id % body->GetName() % link->GetName() % suffix);
}

LinkPtr LinkMarker::link() const
{
    return link_.lock();
}

void LinkMarker::set_pose(OpenRAVE::Transform const &pose) const
{
    server_->setPose(interactive_marker_->name, toROSPose(pose),
                     interactive_marker_->header);
}

void LinkMarker::clear_color()
{
    force_update_ = force_update_ || !!override_color_;
    override_color_.reset();
}

void LinkMarker::set_color(OpenRAVE::Vector const &color)
{
    force_update_ = force_update_ || !override_color_
                                  || (color[0] != (*override_color_)[0])
                                  || (color[1] != (*override_color_)[1])
                                  || (color[2] != (*override_color_)[2])
                                  || (color[3] != (*override_color_)[3]);
    override_color_.reset(color);
}

bool LinkMarker::is_view_visual() const
{
    return view_visual_;
}

void LinkMarker::set_view_visual(bool flag)
{
    force_update_ = force_update_ || (flag != view_visual_);
    view_visual_ = flag;
}

bool LinkMarker::is_view_collision() const
{
    return view_collision_;
}

void LinkMarker::set_view_collision(bool flag)
{
    force_update_ = force_update_ || (flag != view_collision_);
    view_collision_ = flag;
}

InteractiveMarkerPtr LinkMarker::interactive_marker()
{
    return interactive_marker_;
}

std::vector<std::string> LinkMarker::group_names() const
{
    OpenRAVE::KinBody::LinkInfo const &link_info = link()->GetInfo();
    auto const range = link_info._mapExtraGeometries | map_keys;
    return std::vector<std::string>(range.begin(), range.end());
}

void LinkMarker::set_parent_frame(std::string const &frame_id)
{
    if (interactive_marker_->header.frame_id != frame_id) {
        interactive_marker_->header.frame_id = frame_id;
        force_update_ = true;
    }
}

void LinkMarker::SwitchGeometryGroup(std::string const &group)
{
    link()->SetGeometriesFromGroup(group);
    force_update_ = true;
}

bool LinkMarker::EnvironmentSync()
{
    LinkPtr const link = this->link();
    bool is_changed = force_update_;

    // Re-create the geometry.
    if (is_changed) {
        CreateGeometry();
        server_->insert(*interactive_marker_);
    }

    force_update_ = false;
    return is_changed;
}

void LinkMarker::Invalidate()
{
    force_update_ = true;
}

void LinkMarker::CreateGeometry()
{
    visual_control_->markers.clear();
    geometry_markers_.clear();

    LinkPtr const link = this->link();

    for (GeometryPtr const geometry : link->GetGeometries()) {
        // Update this geometry's visibility status.
        visibility_map_[geometry.get()] = geometry->IsVisible();

        // Note that this inserts an empty vector if the entry does not already
        // exist. We depend on this for lazy marker creation.
        std::vector<visualization_msgs::Marker *> &markers
                = geometry_markers_[geometry.get()];

        if (view_visual_ && geometry->IsVisible()) {
            // Try loading the visual mesh.
            MarkerPtr visual_marker = CreateVisualGeometry(geometry);

            // Otherwise, fall back on the collision geometry. This mimics the
            // behavior of qtcoin.
            if (!visual_marker) {
                visual_marker = CreateCollisionGeometry(geometry);
            }

            if (visual_marker) {
                visual_control_->markers.push_back(*visual_marker);
                markers.push_back(&visual_control_->markers.back());
            }
        }

        if (view_collision_ && link->IsEnabled()) {
            MarkerPtr const collision_marker = CreateCollisionGeometry(geometry);

            // Make the collision geometry partially transparent if we're also
            // rendering the collision geometry. It's generally true that the
            // collision geometry is larger than the visual geometry.
            if (collision_marker && view_visual_) {
                collision_marker->color = toROSColor(kCollisionColor);
                collision_marker->mesh_use_embedded_materials = false;
            }

            if (collision_marker) {
                visual_control_->markers.push_back(*collision_marker);
                markers.push_back(&visual_control_->markers.back());
            }
        }
    }
}

MarkerPtr LinkMarker::CreateVisualGeometry(GeometryPtr geometry)
{
    MarkerPtr marker = boost::make_shared<Marker>();
    marker->pose = toROSPose(geometry->GetTransform());

    if (override_color_) {
        marker->color = toROSColor(*override_color_);
    } else {
        marker->color = toROSColor(geometry->GetDiffuseColor());
        marker->color.a = 1.0 - geometry->GetTransparency();
    }

    // If a render filename is specified, then we should ignore the rest of the
    // geometry. This is true regardless of the mesh type.
    std::string render_mesh_path = geometry->GetRenderFilename();
    if (boost::algorithm::starts_with(render_mesh_path, "__norenderif__")) {
        render_mesh_path = "";
    }

    // Pass the path to the mesh to RViz and let RViz load it directly. This is
    // only possible if RViz supports the mesh format.
    if (!render_mesh_path.empty() && HasRVizSupport(render_mesh_path)) {
        marker->type = Marker::MESH_RESOURCE;
        marker->scale = toROSVector(geometry->GetRenderScale());
        marker->mesh_resource = "file://" + render_mesh_path;

        bool const has_texture = !override_color_ && HasTexture(render_mesh_path);
        marker->mesh_use_embedded_materials = has_texture;

        // Color must be zero to use the embedded material.
        if (has_texture) {
            marker->color.r = 0;
            marker->color.g = 0;
            marker->color.b = 0;
            marker->color.a = 0;
        }
        return marker;
    }
    // Otherwise, load the mesh with OpenRAVE and serialize the full mesh it
    // into the marker.
    else if (!render_mesh_path.empty()) {
        OpenRAVE::EnvironmentBasePtr const env = link()->GetParent()->GetEnv();
        TriMeshPtr trimesh = boost::make_shared<OpenRAVE::TriMesh>();
        trimesh = env->ReadTrimeshURI(trimesh, render_mesh_path);
        if (trimesh) {
            TriMeshToMarker(*trimesh, marker);
            marker->scale = toROSVector(geometry->GetInfo()._vCollisionScale);

            static bool already_printed = false;
            if (!already_printed) {
                RAVELOG_WARN("Loaded one or more meshes OpenRAVE because this"
                             " format is not" " supported by RViz. This may be"
                             " slow for large files.\n");
                already_printed = true;
            }
            return marker;
        } else {
            RAVELOG_WARN("Loading trimesh '%s' using OpenRAVE failed.",
                render_mesh_path.c_str()
            );
        }
    }
    return MarkerPtr();
}

MarkerPtr LinkMarker::CreateCollisionGeometry(GeometryPtr geometry)
{
    MarkerPtr marker = boost::make_shared<Marker>();
    marker->pose = toROSPose(geometry->GetTransform());

    if (override_color_) {
        marker->color = toROSColor(*override_color_);
    } else {
        marker->color = toROSColor(geometry->GetDiffuseColor());
        marker->color.a = 1.0 - geometry->GetTransparency();
    }

    switch (geometry->GetType()) {
    case OpenRAVE::GeometryType::GT_None:
        return MarkerPtr();

    case OpenRAVE::GeometryType::GT_Box:
        // TODO: This may be off by a factor of two.
        marker->type = Marker::CUBE;
        marker->scale = toROSVector(geometry->GetBoxExtents());
        marker->scale.x *= 2.0;
        marker->scale.y *= 2.0;
        marker->scale.z *= 2.0;
        if (marker->scale.x * marker->scale.y * marker->scale.z == 0.0) {
            return MarkerPtr();
        }
        break;

    case OpenRAVE::GeometryType::GT_Sphere: {
        double const sphere_radius = geometry->GetSphereRadius();
        marker->type = Marker::SPHERE;
        marker->scale.x = 2.0 * sphere_radius;
        marker->scale.y = 2.0 * sphere_radius;
        marker->scale.z = 2.0 * sphere_radius;
        if (sphere_radius == 0.0) {
            return MarkerPtr();
        }
        break;
    }

    case OpenRAVE::GeometryType::GT_Cylinder: {
        // TODO: This may be rotated and/or off by a factor of two.
        double const cylinder_radius = geometry->GetCylinderRadius();
        double const cylinder_height= geometry->GetCylinderHeight();
        marker->type = Marker::CYLINDER;
        marker->scale.x = 2.0 * cylinder_radius;
        marker->scale.y = 2.0 * cylinder_radius;
        marker->scale.z = cylinder_height;
        break;
    }

    case OpenRAVE::GeometryType::GT_TriMesh:
        TriMeshToMarker(geometry->GetCollisionMesh(), marker);
        break;

    default:
        RAVELOG_WARN("Unknown geometry type '%d' for link '%s'.\n",
            geometry->GetType(), link()->GetName().c_str()
        );
        return MarkerPtr();
    }
    return marker;
}

void LinkMarker::TriMeshToMarker(OpenRAVE::TriMesh const &trimesh,
                                 MarkerPtr const &marker)
{
    marker->type = Marker::TRIANGLE_LIST;
    marker->points.clear();

    BOOST_ASSERT(trimesh.indices.size() % 3 == 0);
    for (size_t i = 0; i < trimesh.indices.size() / 3; ++i) {
        int const index1 = trimesh.indices.at(3 * i + 0);
        int const index2 = trimesh.indices.at(3 * i + 1);
        int const index3 = trimesh.indices.at(3 * i + 2);
        OpenRAVE::Vector const &p1 = trimesh.vertices.at(index1);
        OpenRAVE::Vector const &p2 = trimesh.vertices.at(index2);
        OpenRAVE::Vector const &p3 = trimesh.vertices.at(index3);
        marker->points.push_back(toROSPoint(p1));
        marker->points.push_back(toROSPoint(p2));
        marker->points.push_back(toROSPoint(p3));
    }

    if (trimesh.indices.empty())
    {
        geometry_msgs::Point const point = toROSPoint(OpenRAVE::Vector());
        marker->points.resize(3, point);
    }
}

bool LinkMarker::HasTexture(std::string const &uri) const
{
    return iends_with(uri, ".dae");
}

bool LinkMarker::HasRVizSupport(std::string const &uri) const
{
    return iends_with(uri, ".dae")
        || iends_with(uri, ".stl")
        || iends_with(uri, ".mesh");
}

}
}
