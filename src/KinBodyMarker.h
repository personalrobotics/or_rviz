#ifndef KINBODYMARKER_H_
#define KINBODYMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include "LinkMarker.h"

namespace or_interactivemarker {

class ManipulatorMarker;

class ManipulatorMarker {
public:
    ManipulatorMarker(OpenRAVE::RobotBase::ManipulatorPtr manipulator) {}
};

typedef boost::shared_ptr<ManipulatorMarker> ManipulatorMarkerPtr;

// ---

class KinBodyMarker;
typedef boost::shared_ptr<KinBodyMarker> KinBodyMarkerPtr;

class KinBodyMarker {
public:
    KinBodyMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                  OpenRAVE::KinBodyPtr kinbody);

    bool IsRobot() const;

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBodyPtr kinbody_;
    OpenRAVE::RobotBasePtr robot_;
    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerPtr> link_markers_;
    boost::unordered_map<OpenRAVE::RobotBase::Manipulator *, ManipulatorMarkerPtr> manipulator_markers_;

    void CreateManipulators();
};

}

#endif
