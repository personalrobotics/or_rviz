#include <boost/make_shared.hpp>
#include "KinBodyMarker.h"

namespace or_interactivemarker {

KinBodyMarker::KinBodyMarker(OpenRAVE::KinBodyPtr kinbody)
    : kinbody_(kinbody)
{
    BOOST_ASSERT(kinbody);
}

void KinBodyMarker::EnvironmentSync()
{
    typedef OpenRAVE::KinBody::LinkPtr LinkPtr;

    std::vector<LinkPtr> const &links = kinbody_->GetLinks();

    for (LinkPtr link : links) {
        LinkMarkerPtr &link_marker = link_markers_[link.get()];
        if (!link_marker) {
            link_marker = boost::make_shared<LinkMarker>(link);
        }
        link_marker->EnvironmentSync();
    }
}

}
