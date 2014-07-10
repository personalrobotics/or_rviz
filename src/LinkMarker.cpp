#include "LinkMarker.h"

namespace or_interactivemarker {

LinkMarker::LinkMarker(OpenRAVE::KinBody::LinkPtr link)
    : link_(link)
{
    BOOST_ASSERT(link);
}

}
