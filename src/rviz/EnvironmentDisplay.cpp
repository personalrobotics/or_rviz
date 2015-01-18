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
#include <boost/format.hpp>
#include <pluginlib/class_list_macros.h>
#include <rviz/display_context.h>
#include "rviz/EnvironmentDisplay.h"
#include "util/ros_conversions.h"

using boost::format;
using boost::str;

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
    property_environment_ = new ::rviz::EnumProperty(
        "Environment", QString(), "OpenRAVE environment",
        this, SLOT(EnvironmentChangeSlot())
    );
    property_frame_ = new ::rviz::TfFrameProperty(
        "World Frame", kDefaultFrame, "OpenRAVE world frame",
        this, context_->getFrameManager(), true,
        SLOT(FrameChangeSlot())
    );
    property_bodies_ = new ::rviz::Property("Bodies", QVariant(), "", this);
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

void EnvironmentDisplay::EnvironmentSync()
{
    // Build a list of all environments IDs.
    std::list<OpenRAVE::EnvironmentBasePtr> envs;
    OpenRAVE::RaveGetEnvironments(envs);

    std::set<int> new_environment_ids;
    for (OpenRAVE::EnvironmentBasePtr const &env : envs) {
        int const id = OpenRAVE::RaveGetEnvironmentId(env);
        new_environment_ids.insert(id);
    }

    // Update the dropdown menu.
    if (new_environment_ids != environment_ids_) {
        property_environment_->clearOptions();
        for (int const &id : new_environment_ids) {
            std::string const label = str(format("%d") % id);
            property_environment_->addOptionStd(label, id);
        }

        int const current_id = property_environment_->getOptionInt();
        if (!new_environment_ids.count(current_id)) {
            RAVELOG_WARN("Currently viewing environment ID %d, which no"
                         " longer exists.\n", current_id);
        }

        environment_ids_ = new_environment_ids;
    }
}

boost::signals2::connection EnvironmentDisplay::RegisterFrameChangeCallback(
    boost::function<FrameChangeCallback> const &callback)
{
    return frame_callbacks_.connect(callback);
}

boost::signals2::connection EnvironmentDisplay::RegisterEnvironmentChangeCallback(
    boost::function<EnvironmentChangeCallback> const &callback)
{
    return env_changed_callbacks_.connect(callback);
}


/*
 * Slots
 */
void EnvironmentDisplay::FrameChangeSlot()
{
    frame_callbacks_(property_frame_->getFrameStd());
}

void EnvironmentDisplay::EnvironmentChangeSlot()
{
    int const id = property_environment_->getOptionInt();
    OpenRAVE::EnvironmentBasePtr changed_env = OpenRAVE::RaveGetEnvironment(id);

    if (changed_env) {
        env_changed_callbacks_(changed_env);
    } else {
        RAVELOG_WARN("Attempted to switch to environment with non-existent"
                     " ID %d.\n", id);
    }
}


/*
 * Private
 */
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
