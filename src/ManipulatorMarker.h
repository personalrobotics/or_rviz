#ifndef MANIPULATORMARKER_H_
#define MANIPULATORMARKER_H_

#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>

namespace or_interactivemarker {

class ManipulatorMarker;
typedef boost::shared_ptr<ManipulatorMarker> ManipulatorMarkerPtr;

class ManipulatorMarker {
public:
    ManipulatorMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                      OpenRAVE::RobotBase::ManipulatorPtr manipulator);

    std::string id() const;

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::RobotBase::ManipulatorPtr manipulator_;

    void IkFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    visualization_msgs::InteractiveMarker ik_marker_;
    visualization_msgs::InteractiveMarkerControl *ik_control_;

    bool changed_pose_;
    OpenRAVE::Transform current_pose_;
    std::vector<OpenRAVE::dReal> current_ik_;
};

}

#endif
