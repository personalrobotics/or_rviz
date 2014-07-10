#include "KinBodyMarker.h"

namespace or_interactivemarker {

KinBodyMarker::KinBodyMarker(OpenRAVE::KinBodyPtr kinbody)
    : kinbody_(kinbody)
{
    BOOST_ASSERT(kinbody);
}

}
