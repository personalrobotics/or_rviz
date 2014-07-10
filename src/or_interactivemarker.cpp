#include <interactive_markers/interactive_marker_server.h>
#include "or_interactivemarker.h"

namespace or_interactivemarker {

InteractiveMarkerViewer::InteractiveMarkerViewer(
        OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::ViewerBase(env)
    , env_(env)
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
    using OpenRAVE::KinBodyPtr;

    // TODO: Do I need to lock here? Is the environment already locked?

    std::vector<KinBodyPtr> bodies;
    env_->GetBodies(bodies);

    for (KinBodyPtr body : bodies) {
        KinBodyMarkerPtr &body_marker = body_markers_[body.get()];
        if (!body_marker) {
            body_marker = boost::make_shared<KinBodyMarker>(body);
        }
        body_marker->EnvironmentSync();
    }
}

}
