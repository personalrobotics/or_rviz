#ifndef KINBODYMARKER_H_
#define KINBODYMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include "LinkMarker.h"

namespace or_interactivemarker {

class KinBodyMarker;
typedef boost::shared_ptr<KinBodyMarker> KinBodyMarkerPtr;

class KinBodyMarker {
public:
    KinBodyMarker(OpenRAVE::KinBodyPtr kinbody);

    void EnvironmentSync();

private:
    OpenRAVE::KinBodyPtr kinbody_;
    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerPtr> link_markers_;
};

}

#endif
