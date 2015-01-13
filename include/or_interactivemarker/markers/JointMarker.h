#ifndef JOINTMARKER_H_
#define JOINTMARKER_H_
#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>

namespace or_interactivemarker {
namespace markers {

class JointMarker;
typedef boost::shared_ptr<JointMarker> JointMarkerPtr;

class JointMarker {
public:
    JointMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                OpenRAVE::KinBody::JointPtr joint);
    virtual ~JointMarker();

    std::string id() const;
    OpenRAVE::KinBody::JointPtr joint() const;

    void set_parent_frame(std::string const &frame_id);

    OpenRAVE::Transform pose() const;
    void set_pose(OpenRAVE::Transform const &pose);

    double angle() const;
    
    void set_joint_pose(OpenRAVE::Transform const &pose);

    virtual bool EnvironmentSync();

    static OpenRAVE::Transform GetJointPose(OpenRAVE::KinBody::JointPtr joint);

protected:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    visualization_msgs::InteractiveMarker marker_;
    visualization_msgs::InteractiveMarkerControl *joint_control_;

private:
    OpenRAVE::KinBody::JointWeakPtr joint_;
    OpenRAVE::Transform joint_pose_;
    double joint_initial_;
    double joint_delta_;
    bool created_;
    bool force_update_;
    bool active_;

    void JointCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
};

}
}

#endif
