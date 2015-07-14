#ifndef INTERACTIVEMARKERGRAPHHANDLE_H_
#define INTERACTIVEMARKERGRAPHHANDLE_H_
#include <boost/function.hpp>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif
#include <interactive_markers/interactive_marker_server.h>

namespace or_rviz {
namespace util {

class InteractiveMarkerGraphHandle : public OpenRAVE::GraphHandle {
public:
    typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

    InteractiveMarkerGraphHandle(
        InteractiveMarkerServerPtr const &interactive_marker_server,
        visualization_msgs::InteractiveMarkerPtr const &interactive_marker,
        boost::function<void (InteractiveMarkerGraphHandle *)> const &callback
    );

    virtual ~InteractiveMarkerGraphHandle();

    void set_parent_frame(std::string const &frame_id);

    virtual void SetTransform(OpenRAVE::RaveTransform<float> const &t);
    virtual void SetShow(bool show);

private:
    InteractiveMarkerServerPtr server_;
    visualization_msgs::InteractiveMarkerPtr interactive_marker_;
    boost::function<void (InteractiveMarkerGraphHandle *)> remove_callback_;
    bool show_;
};

typedef boost::shared_ptr<InteractiveMarkerGraphHandle> InteractiveMarkerGraphHandlePtr;

}}

#endif
