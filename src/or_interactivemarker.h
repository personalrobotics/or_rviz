#ifndef ORINTERACTIVEMARKER_H_
#define ORINTERACTIVEMARKER_H_
#include <boost/unordered_map.hpp>
#include <openrave/openrave.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include "KinBodyMarker.h"

namespace or_interactivemarker {

class InteractiveMarkerViewer : public OpenRAVE::ViewerBase {
public:
    InteractiveMarkerViewer(OpenRAVE::EnvironmentBasePtr env);

    virtual void EnvironmentSync();

    virtual int main(bool bShow = true);
    virtual void quitmainloop();

private:
    boost::unordered_map<std::string, KinBodyMarkerPtr> body_markers_;
};

}

#endif
