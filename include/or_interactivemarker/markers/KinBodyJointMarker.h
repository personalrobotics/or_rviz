#ifndef KINBODYJOINTMARKER_H_
#define KINBODYJOINTMARKER_H_
#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>
#include "JointMarker.h"

namespace or_interactivemarker {
namespace markers {

class KinBodyJointMarker;
typedef boost::shared_ptr<KinBodyJointMarker> KinBodyJointMarkerPtr;

class KinBodyJointMarker : public JointMarker {
public:
    KinBodyJointMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                       OpenRAVE::KinBody::JointPtr joint);
    virtual ~KinBodyJointMarker();

    virtual bool EnvironmentSync();
};

}
}

#endif
