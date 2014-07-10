#include <boost/make_shared.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include "or_interactivemarker.h"

using interactive_markers::InteractiveMarkerServer;

using OpenRAVE::KinBodyPtr;

namespace or_interactivemarker {

InteractiveMarkerViewer::InteractiveMarkerViewer(
        OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::ViewerBase(env)
    , env_(env)
    , server_(boost::make_shared<InteractiveMarkerServer>("openrave"))
{
    BOOST_ASSERT(env);

}

int InteractiveMarkerViewer::main(bool bShow)
{
}

void InteractiveMarkerViewer::quitmainloop()
{
}

void InteractiveMarkerViewer::EnvironmentSync()
{
    // TODO: Do I need to lock here? Is the environment already locked?
    OpenRAVE::EnvironmentMutex::scoped_lock lock(env_->GetMutex(),
                                                 boost::try_to_lock);
    if (!lock) {
        return;
    }

    std::vector<KinBodyPtr> bodies;
    env_->GetBodies(bodies);

    for (KinBodyPtr body : bodies) {
        KinBodyMarkerPtr &body_marker = body_markers_[body.get()];
        if (!body_marker) {
            body_marker = boost::make_shared<KinBodyMarker>(server_, body);
        }
        body_marker->EnvironmentSync();
    }

    server_->applyChanges();
}

}
