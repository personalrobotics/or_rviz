#include <sstream>
#include <string>
#include <ros/ros.h>
#include <openrave/plugin.h>
#include "or_interactivemarker.h"

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream & sinput, EnvironmentBasePtr env)
{
    using namespace or_interactivemarker;

    if (type == PT_Viewer && interfacename == "interactivemarker") {
        std::string node_name;
        sinput >> node_name;

        if (sinput.fail()) {
            return OpenRAVE::InterfaceBasePtr();
        }

        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
            RAVELOG_DEBUG("Starting ROS node '%s'.\n", node_name.c_str());
        } else {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n",
                          ros::this_node::getName().c_str());
        }

        return InterfaceBasePtr(new InteractiveMarkerViewer(env));
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[OpenRAVE::PT_Viewer].push_back("InteractiveMarker");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
