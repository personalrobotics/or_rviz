#ifndef SCOPEDCONNECTION_H_
#define SCOPEDCONNECTION_H_
#include <boost/signals2.hpp>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif

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
