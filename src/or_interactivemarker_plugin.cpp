#include <QApplication>
#include <boost/make_shared.hpp>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <openrave/plugin.h>
#include "or_interactivemarker.h"
#include "or_rviz.h"

static std::string const kDefaultTopicName = "openrave";

static QApplication *qt_application = NULL;
static char *qt_argv[1] = { const_cast<char *>("or_rviz") };
static int qt_argc = 1;

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream & sinput, EnvironmentBasePtr env)
{
    using namespace or_interactivemarker;

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
