#include <boost/make_shared.hpp>
#include "KinBodyMarker.h"

using OpenRAVE::KinBodyPtr;
using OpenRAVE::RobotBase;
using interactive_markers::InteractiveMarkerServer;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

namespace or_interactivemarker {

KinBodyMarker::KinBodyMarker(InteractiveMarkerServerPtr server,
                             KinBodyPtr kinbody)
    : server_(server)
    , kinbody_(kinbody)
    , robot_(boost::dynamic_pointer_cast<RobotBase>(kinbody))
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(kinbody);
}

bool KinBodyMarker::IsRobot() const
{
    return !!robot_;
}

void KinBodyMarker::EnvironmentSync()
{
    typedef OpenRAVE::KinBody::LinkPtr LinkPtr;

    std::vector<LinkPtr> const &links = kinbody_->GetLinks();

    for (LinkPtr link : links) {
        LinkMarkerPtr &link_marker = link_markers_[link.get()];
        if (!link_marker) {
            link_marker = boost::make_shared<LinkMarker>(server_, link);
        }
        link_marker->EnvironmentSync();
    }
}

void KinBodyMarker::CreateManipulators()
{
    if (!robot_) {
        return;
    }

    for (ManipulatorPtr const manipulator : robot_->GetManipulators()) {
        manipulator_markers_[manipulator.get()]
            = boost::make_shared<ManipulatorMarker>(manipulator);
    }
}

}
