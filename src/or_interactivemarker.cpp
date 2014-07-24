#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include "or_interactivemarker.h"

using boost::format;
using boost::str;
using interactive_markers::InteractiveMarkerServer;

using OpenRAVE::KinBodyPtr;

static double const kRefreshRate = 30;

namespace or_interactivemarker {

namespace detail {

class ScopedConnection : public OpenRAVE::UserData {
public:
    ScopedConnection(boost::signals2::connection const &connection)
        : scoped_connection_(connection)
    {
    }

    virtual ~ScopedConnection()
    {
    }

private:
    boost::signals2::scoped_connection scoped_connection_;
};

static std::string GetRemainingContent(std::istream &stream, bool trim = false)
{

    std::istreambuf_iterator<char> eos;
    std::string str(std::istreambuf_iterator<char>(stream), eos);
    if (trim) {
        boost::algorithm::trim(str);
    }
    return str;
}

}


InteractiveMarkerViewer::InteractiveMarkerViewer(
        OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::ViewerBase(env)
    , env_(env)
    , server_(boost::make_shared<InteractiveMarkerServer>("openrave"))
    , running_(false)
{
    BOOST_ASSERT(env);

    RegisterCommand("AddMenuEntry",
        boost::bind(&InteractiveMarkerViewer::AddMenuEntryCommand, this, _1, _2),
        "Attach a custom menu entry to an object."
    );
    RegisterCommand("GetMenuSelection",
        boost::bind(&InteractiveMarkerViewer::GetMenuSelectionCommand, this, _1, _2),
        "Get the name of the last menu selection."
    );
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
        viewer_callbacks_();
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

OpenRAVE::UserDataPtr InteractiveMarkerViewer::RegisterItemSelectionCallback(
    OpenRAVE::ViewerBase::ItemSelectionCallbackFn const &fncallback)
{
    boost::signals2::connection const con = selection_callbacks_.connect(fncallback);
    return boost::make_shared<detail::ScopedConnection>(con);
}

OpenRAVE::UserDataPtr InteractiveMarkerViewer::RegisterViewerThreadCallback(
    OpenRAVE::ViewerBase::ViewerThreadCallbackFn const &fncallback)
{
    boost::signals2::connection const con = viewer_callbacks_.connect(fncallback);
    return boost::make_shared<detail::ScopedConnection>(con);
}

bool InteractiveMarkerViewer::AddMenuEntryCommand(std::ostream &out, std::istream &in)
{
    std::string type, kinbody_name;
    in >> type >> kinbody_name;

    // Get the KinBodyMarker associated with the target object.
    OpenRAVE::KinBodyPtr const kinbody = env_->GetKinBody(kinbody_name);
    if (!kinbody) {
        throw OpenRAVE::openrave_exception(
            str(format("There is no KinBody named '%s' in the environment.")
                % kinbody_name),
            OpenRAVE::ORE_Failed
        );
    }

    OpenRAVE::UserDataPtr const marker_raw = kinbody->GetUserData("interactive_marker"); 
    auto const marker = boost::dynamic_pointer_cast<KinBodyMarker>(marker_raw);
    if (!marker) {
        throw OpenRAVE::openrave_exception(
            str(format("KinBody '%s' does not have an associated marker.")
                %  kinbody_name),
            OpenRAVE::ORE_InvalidState
        );
    }

    if (type == "kinbody") {
        std::string const name = detail::GetRemainingContent(in, true);
        auto const callback = boost::bind(
            &InteractiveMarkerViewer::KinBodyMenuCallback,
            this, kinbody, name
        );
        marker->AddMenuEntry(name, callback);
    } else if (type == "link") {
        std::string link_name;
        in >> link_name;

        OpenRAVE::KinBody::LinkPtr const link = kinbody->GetLink(link_name);
        if (!link) {
            throw OpenRAVE::openrave_exception(
                str(format("KinBody '%s' has no link '%s'.")
                    % kinbody_name % link_name),
                OpenRAVE::ORE_Failed
            );
        }

        std::string const name = detail::GetRemainingContent(in, true);
        auto const callback = boost::bind(
            &InteractiveMarkerViewer::LinkMenuCallback,
            this, link, name
        );
        marker->AddMenuEntry(link, name, callback);
    } else if (type == "manipulator" || type == "ghost_manipulator") {
        std::string manipulator_name;
        in >> manipulator_name;

        if (!kinbody->IsRobot()) {
            throw OpenRAVE::openrave_exception(
                str(format("KinBody '%s' is not a robot and does not support"
                           " manipulator menus.")
                    % kinbody_name),
                OpenRAVE::ORE_Failed
            );
        }

        auto const robot = boost::dynamic_pointer_cast<OpenRAVE::RobotBase>(kinbody);
        OpenRAVE::RobotBase::ManipulatorPtr const manipulator = robot->GetManipulator(manipulator_name);
        if (!manipulator) {
            throw OpenRAVE::openrave_exception(
                str(format("Robot '%s' has no manipulator '%s'.")
                    % kinbody_name % manipulator_name),
                OpenRAVE::ORE_Failed
            );
        }

        std::string const name = detail::GetRemainingContent(in, true);
        auto const callback = boost::bind(
            &InteractiveMarkerViewer::ManipulatorMenuCallback,
            this, manipulator, name
        );
        marker->AddMenuEntry(manipulator, name, callback);
    }
    return true;
}

bool InteractiveMarkerViewer::GetMenuSelectionCommand(std::ostream &out,
                                                      std::istream &in)
{
    out << menu_queue_.rdbuf();
}

void InteractiveMarkerViewer::KinBodyMenuCallback(OpenRAVE::KinBodyPtr kinbody,
                                                  std::string const &name)
{
    menu_queue_ << "kinbody " << kinbody->GetName()
                << " " << name << '\n';
    selection_callbacks_(OpenRAVE::KinBody::LinkPtr(),
                         OpenRAVE::RaveVector<float>(),
                         OpenRAVE::RaveVector<float>());
}

void InteractiveMarkerViewer::LinkMenuCallback(OpenRAVE::KinBody::LinkPtr link,
                                               std::string const &name)
{
    menu_queue_ << "link " << link->GetParent()->GetName()
                << " " << link->GetName()
                << " " << name << '\n';
}

void InteractiveMarkerViewer::ManipulatorMenuCallback(
        OpenRAVE::RobotBase::ManipulatorPtr manipulator, std::string const &name)
{
    menu_queue_ << "manipulator " << manipulator->GetRobot()->GetName()
                << " " << manipulator->GetName()
                << " " << name << '\n';
}

}
