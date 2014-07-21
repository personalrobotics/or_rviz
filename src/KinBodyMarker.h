#ifndef KINBODYMARKER_H_
#define KINBODYMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include "LinkMarker.h"
#include "JointMarker.h"
#include "ManipulatorMarker.h"

namespace or_interactivemarker {

class KinBodyMarker;
typedef boost::shared_ptr<KinBodyMarker> KinBodyMarkerPtr;

class KinBodyMarker : public OpenRAVE::UserData {
public:
    KinBodyMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                  OpenRAVE::KinBodyPtr kinbody);
    virtual ~KinBodyMarker();

    bool IsGhost() const;

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBodyWeakPtr kinbody_;
    OpenRAVE::RobotBaseWeakPtr robot_;

    OpenRAVE::KinBodyPtr ghost_kinbody_;
    OpenRAVE::RobotBasePtr ghost_robot_;

    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerPtr> link_markers_;
    boost::unordered_map<OpenRAVE::KinBody::Joint *, JointMarkerPtr> joint_markers_;
    boost::unordered_map<OpenRAVE::RobotBase::Manipulator *, ManipulatorMarkerPtr> manipulator_markers_;

    void CreateGhost();
};

}

#endif
