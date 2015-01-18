/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Matthew Klingensmith <mklingen@cs.cmu.edu>

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
#include <QVariant>
#include <QString>
#include <pluginlib/class_list_macros.h>
#include <rviz/display_context.h>
#include "rviz/EnvironmentDisplay.h"
#include "util/ros_conversions.h"

static QString const kDefaultFrame = "map";

namespace or_interactivemarker {
namespace rviz {

EnvironmentDisplay::EnvironmentDisplay()
{
}

EnvironmentDisplay::~EnvironmentDisplay()
{
}

void EnvironmentDisplay::onInitialize()
{
    property_bodies_ = new ::rviz::Property("Bodies", QVariant(), "", this);
    property_frame_ = new ::rviz::TfFrameProperty(
        "World Frame", kDefaultFrame, "OpenRAVE world frame",
        this, context_->getFrameManager(), true,
        SLOT(FrameChangeSlot())
    );
}

void EnvironmentDisplay::set_environment(OpenRAVE::EnvironmentBasePtr const &env)
{
    // Listen for bodies added and removed from the environment.
    env_ = env;
    env_callback_handle_ = env->RegisterBodyCallback(
        boost::bind(&EnvironmentDisplay::BodyCallback, this, _1, _2)
    );

    // Manually add the existing set of bodies in the environment. The callback
    // will not be called for these because they already exist.
    body_properties_.clear();

    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);

    for (OpenRAVE::KinBodyPtr const &body : bodies) {
        BodyCallback(body, 1);
    }

    // TODO: Should we do this here?
    frame_callbacks_(property_frame_->getFrameStd());
}

boost::signals2::connection EnvironmentDisplay::RegisterFrameChangeCallback(
    boost::function<FrameChangeCallback> const &callback)
{
    return frame_callbacks_.connect(callback);
}

void EnvironmentDisplay::FrameChangeSlot()
{
    frame_callbacks_(property_frame_->getFrameStd());
}

void EnvironmentDisplay::BodyCallback(OpenRAVE::KinBodyPtr body, int flag)
{
    // Body was remove.
    if (flag == 1) {
        body_properties_[body.get()] = new ::rviz::Property(
            QString::fromStdString(body->GetName()),
            QVariant(), "", property_bodies_
        );
    }
    // Body was added.
    else if (flag == 0) {
        auto it = body_properties_.find(body.get());

        if (it != body_properties_.end()) {
            delete property_bodies_->takeChild(it->second);
            body_properties_.erase(it);
        } else {
            RAVELOG_WARN("Received removal event for unknown body '%s'.\n",
                body->GetName().c_str()
            );
        }
    } else {
        RAVELOG_WARN("Unkonwn RegisterBodyCallback flag %d.\n", flag);
    }
}

}
}

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
PLUGINLIB_EXPORT_CLASS(or_interactivemarker::rviz::EnvironmentDisplay,
                       ::rviz::Display)
