#include <boost/make_shared.hpp>
#include "KinBodyMarker.h"

using interactive_markers::InteractiveMarkerServer;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

namespace or_interactivemarker {

KinBodyMarker::KinBodyMarker(InteractiveMarkerServerPtr server,
                             OpenRAVE::KinBodyPtr kinbody)
    : server_(server)
    , kinbody_(kinbody)
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(kinbody);
}

void KinBodyMarker::EnvironmentSync()
{
    typedef OpenRAVE::KinBody::LinkPtr LinkPtr;

    std::vector<LinkPtr> const &links = kinbody_->GetLinks();

    for (LinkPtr link : links) {
        LinkMarkerPtr &link_marker = link_markers_[link.get()];
        if (!link_marker) {
            link_marker = boost::make_shared<LinkMarker>(server_, link);
            RAVELOG_DEBUG("Created LinkMarker for '%s'.\n", link->GetName().c_str());
        }
        link_marker->EnvironmentSync();
    }
}

}
