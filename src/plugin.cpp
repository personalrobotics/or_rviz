/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Matthew Klingensmith <mklingen@cs.cmu.edu>
         Michael Koval <mkoval@cs.cmu.edu>

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
#include <QApplication>
#include <boost/make_shared.hpp>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <openrave/plugin.h>
#include "InteractiveMarkerViewer.h"
#include "RVizViewer.h"

static std::string const kDefaultTopicName = "openrave";

static QApplication *qt_application = NULL;
static char *qt_argv[1] = { const_cast<char *>("or_rviz") };
static int qt_argc = 1;

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream & sinput, EnvironmentBasePtr env)
{
    using namespace or_rviz;

    if (type == PT_Viewer && (interfacename == "interactivemarker"
                           || interfacename == "rviz")) {
        std::string node_name;
        sinput >> node_name;

        if (sinput.fail()) {
            // Default node name.
            node_name = "openrave";
        }

        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, NULL, node_name, ros::init_options::AnonymousName
                                           | ros::init_options::NoSigintHandler);
            RAVELOG_DEBUG("Starting ROS node '%s'.\n", node_name.c_str());
        } else {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n",
                          ros::this_node::getName().c_str());
        }

        if (interfacename == "interactivemarker") {
            return boost::make_shared<InteractiveMarkerViewer>(env, kDefaultTopicName);
        } else if (interfacename == "rviz") {
            // Use one, global QApplication for all or_rviz windows.
            // TODO: Does this need to be re-created if the viewer is closed?
            if (!qt_application) {
                qt_application =  new QApplication(qt_argc, qt_argv);
            }
            
            return boost::make_shared<RVizViewer>(env, kDefaultTopicName, true);
        } else {
            // This should never happen.
            BOOST_ASSERT(false);
            return InterfaceBasePtr();
        }
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[OpenRAVE::PT_Viewer].push_back("RViz");
    info.interfacenames[OpenRAVE::PT_Viewer].push_back("InteractiveMarker");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
