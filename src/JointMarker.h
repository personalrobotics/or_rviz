#ifndef JOINTMARKER_H_
#define JOINTMARKER_H_

#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>

namespace or_interactivemarker {

class JointMarker;
typedef boost::shared_ptr<JointMarker> JointMarkerPtr;

class JointMarker {
public:
    JointMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                OpenRAVE::KinBody::JointPtr joint);
    virtual ~JointMarker();

    std::string id() const;

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBody::JointPtr joint_;

    visualization_msgs::InteractiveMarker marker_;
    visualization_msgs::InteractiveMarkerControl *joint_control_;
};

}

#endif
