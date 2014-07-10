#include <interactive_markers/interactive_marker_server.h>
#include "or_interactivemarker.h"

namespace or_interactivemarker {

InteractiveMarkerViewer::InteractiveMarkerViewer(
        OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::ViewerBase(env)
{
}

int InteractiveMarkerViewer::main(bool bShow)
{
}

void InteractiveMarkerViewer::quitmainloop()
{
}

void InteractiveMarkerViewer::EnvironmentSync()
{
}

}
