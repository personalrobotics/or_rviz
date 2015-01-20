#include "util/ScopedConnection.h"

namespace or_interactivemarker {
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
