#ifndef MANIPULATORMARKER_H_
#define MANIPULATORMARKER_H_
#include <boost/unordered_map.hpp>
#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>
#include "LinkMarker.h"
#include "JointMarker.h"

namespace or_interactivemarker {

class ManipulatorMarker;
typedef boost::shared_ptr<ManipulatorMarker> ManipulatorMarkerPtr;

class ManipulatorMarker {
public:
    ManipulatorMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                      OpenRAVE::RobotBase::ManipulatorPtr manipulator);
    virtual ~ManipulatorMarker();

    std::string id() const;
    bool EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::RobotBase::ManipulatorPtr manipulator_;

    visualization_msgs::InteractiveMarker ik_marker_;
    visualization_msgs::InteractiveMarkerControl *ik_control_;
    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerPtr> link_markers_;
    boost::unordered_map<OpenRAVE::KinBody::Joint *, JointMarkerPtr> free_joint_markers_;

    bool changed_pose_;
    OpenRAVE::Transform current_pose_;
    std::vector<OpenRAVE::dReal> current_ik_;
    std::vector<OpenRAVE::dReal> current_free_;

    void CreateGeometry();
    void IkFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
    void InferFreeJoints(std::vector<OpenRAVE::KinBody::JointPtr> *free_joints) const;
};

}

#endif
