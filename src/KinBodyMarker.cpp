#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include "KinBodyMarker.h"
#include "or_conversions.h"

using boost::format;
using boost::str;
using visualization_msgs::InteractiveMarker;
using interactive_markers::InteractiveMarkerServer;

typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

namespace or_interactivemarker {

KinBodyMarker::KinBodyMarker(InteractiveMarkerServerPtr server,
                             OpenRAVE::KinBodyPtr kinbody)
    : server_(server)
    , kinbody_(kinbody)
    , interactive_marker_(boost::make_shared<InteractiveMarker>())
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(kinbody);

    interactive_marker_->header.frame_id = kWorldFrameId;
    interactive_marker_->name = id();
    interactive_marker_->description = kinbody->GetName();
    interactive_marker_->scale = 0.25;

    CreateControls();
    EnvironmentSync();

    server_->insert(*interactive_marker_);
}

std::string KinBodyMarker::id() const
{
    OpenRAVE::EnvironmentBasePtr const env = kinbody_->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);
    return str(format("Environment[%d].KinBody[%s]")
               % environment_id % kinbody_->GetName());
}

void KinBodyMarker::EnvironmentSync()
{
    interactive_marker_->pose = toROSPose(kinbody_->GetTransform());

    std::vector<LinkPtr> const &links = kinbody_->GetLinks();
    for (LinkPtr const link : links) {
        auto const it = link_markers_.find(link.get());
        BOOST_ASSERT(it != link_markers_.end());
        it->second->EnvironmentSync();
    }
}

void KinBodyMarker::CreateControls()
{
    std::vector<LinkPtr> const &links = kinbody_->GetLinks();

    for (LinkPtr link : links) {
        LinkMarkerPtr &link_marker = link_markers_[link.get()];
        link_marker = boost::make_shared<LinkMarker>(server_, link);

        interactive_marker_->controls.insert(
            interactive_marker_->controls.end(),
            link_marker->begin(), link_marker->end()
        );
    }
}

}
