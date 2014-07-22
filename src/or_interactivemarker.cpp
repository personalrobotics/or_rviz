#include <boost/make_shared.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include "or_interactivemarker.h"

using interactive_markers::InteractiveMarkerServer;

using OpenRAVE::KinBodyPtr;

static double const kRefreshRate = 30;

namespace or_interactivemarker {

InteractiveMarkerViewer::InteractiveMarkerViewer(
        OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::ViewerBase(env)
    , env_(env)
    , server_(boost::make_shared<InteractiveMarkerServer>("openrave"))
    , running_(false)
{
    BOOST_ASSERT(env);

}

int InteractiveMarkerViewer::main(bool bShow)
{
    ros::Rate rate(kRefreshRate);

    RAVELOG_DEBUG("Starting main loop with a %.0f Hz refresh rate.\n",
        kRefreshRate
    );

    running_ = true;
    while (running_) {
        EnvironmentSync();
        rate.sleep();
    }

    RAVELOG_DEBUG("Exiting main loop.\n");
    return 0;
}

void InteractiveMarkerViewer::quitmainloop()
{
    RAVELOG_DEBUG("Stopping main loop on the cycle (within %.3f ms).\n",
        1.0 / kRefreshRate
    );
    running_ = false;
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
        OpenRAVE::UserDataPtr const raw = body->GetUserData("interactive_marker"); 
        auto body_marker = boost::dynamic_pointer_cast<KinBodyMarker>(raw);
        BOOST_ASSERT(!raw || body_marker);

        // Create the new geometry if neccessary.
        if (!raw) {
            RAVELOG_DEBUG("Creating KinBodyMarker for '%s'.\n",
                body->GetName().c_str()
            );
            body_marker = boost::make_shared<KinBodyMarker>(server_, body);
            body->SetUserData("interactive_marker", body_marker);
        }

        body_marker->EnvironmentSync();
    }
    server_->applyChanges();
    ros::spinOnce();
}

OpenRAVE::UserDataPtr InteractiveMarkerViewer::RegisterViewerThreadCallback(
    OpenRAVE::ViewerBase::ViewerThreadCallbackFn const &fncallback)
{
    RAVELOG_WARN("RegisterViewerThreadCallback is not implemented.\n");
    return OpenRAVE::UserDataPtr();
}

}
