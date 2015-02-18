#ifndef SCOPEDCONNECTION_H_
#define SCOPEDCONNECTION_H_
#include <boost/signals2.hpp>
#include <openrave/openrave.h>

namespace or_rviz {
namespace util {

class ScopedConnection : public OpenRAVE::UserData {
public:
    ScopedConnection(boost::signals2::connection const &connection);
    virtual ~ScopedConnection();

private:
    boost::signals2::scoped_connection scoped_connection_;
};

}
}

#endif
