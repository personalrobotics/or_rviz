#include "util/ScopedConnection.h"

namespace or_rviz {
namespace util {

ScopedConnection::ScopedConnection(boost::signals2::connection const &connection)
    : scoped_connection_(connection)
{
}

ScopedConnection::~ScopedConnection()
{
}

}
}
