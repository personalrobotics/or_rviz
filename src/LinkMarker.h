#ifndef LINKMARKER_H_
#define LINKMARKER_H_
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>

namespace or_interactivemarker {

class LinkMarker;
typedef boost::shared_ptr<LinkMarker> LinkMarkerPtr;

class LinkMarker {
public:
    LinkMarker(OpenRAVE::KinBody::LinkPtr link);

private:
    OpenRAVE::KinBody::LinkPtr link_;
};

}

#endif
