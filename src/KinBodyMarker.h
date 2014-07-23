#ifndef KINBODYMARKER_H_
#define KINBODYMARKER_H_
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>
#include "KinBodyLinkMarker.h"
#include "KinBodyJointMarker.h"
#include "ManipulatorMarker.h"

namespace or_interactivemarker {

class KinBodyMarker;
typedef boost::shared_ptr<KinBodyMarker> KinBodyMarkerPtr;

struct LinkMarkerWrapper {
    typedef interactive_markers::MenuHandler::EntryHandle MenuEntry;

    LinkMarkerWrapper() : has_menu(false) { }

    KinBodyLinkMarkerPtr link_marker;
    OpenRAVE::RobotBase::ManipulatorPtr parent_manipulator;
    bool has_menu;
    boost::optional<MenuEntry> menu_parent;
    boost::optional<MenuEntry> menu_joints;
    boost::optional<MenuEntry> menu_enabled;
    boost::optional<MenuEntry> menu_visible;
    boost::optional<MenuEntry> menu_move;
    boost::optional<MenuEntry> menu_manipulator;
    boost::optional<MenuEntry> menu_manipulator_active;
    boost::optional<MenuEntry> menu_manipulator_joints;
    boost::optional<MenuEntry> menu_manipulator_ik;
};

struct CustomMenuEntry {
    std::string name;
    OpenRAVE::KinBody::LinkWeakPtr link;
    OpenRAVE::RobotBase::ManipulatorWeakPtr manipulator;
    boost::function<void ()> callback;
};

class KinBodyMarker : public OpenRAVE::UserData {
public:
    KinBodyMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                  OpenRAVE::KinBodyPtr kinbody);
    virtual ~KinBodyMarker();

    std::string id() const;

    void AddMenuEntry(std::string const &name, boost::function<void ()> const &callback);
    void AddMenuEntry(OpenRAVE::KinBody::LinkPtr link,
                      std::string const &name, boost::function<void ()> const &callback);
    void AddMenuEntry(OpenRAVE::RobotBase::ManipulatorPtr manipulator,
                      std::string const &name, boost::function<void ()> const &callback);

    void EnvironmentSync();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::KinBodyWeakPtr kinbody_;
    OpenRAVE::RobotBaseWeakPtr robot_;
    bool has_pose_controls_;
    bool has_joint_controls_;

    visualization_msgs::InteractiveMarkerPtr interactive_marker_;
    bool new_marker_;

    std::vector<CustomMenuEntry> menu_custom_kinbody_;
    boost::unordered_map<OpenRAVE::KinBody::Link *, std::vector<CustomMenuEntry> > menu_custom_links_;
    boost::unordered_map<OpenRAVE::RobotBase::Manipulator *, std::vector<CustomMenuEntry> > menu_custom_manipulators_;

    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerWrapper> link_markers_;
    boost::unordered_map<OpenRAVE::KinBody::Joint *, KinBodyJointMarkerPtr> joint_markers_;
    boost::unordered_map<OpenRAVE::RobotBase::Manipulator *, ManipulatorMarkerPtr> manipulator_markers_;

    bool HasGhostManipulator(OpenRAVE::RobotBase::ManipulatorPtr const manipulator) const;

    void CreateMenu(LinkMarkerWrapper &link_wrapper);
    void UpdateMenu(LinkMarkerWrapper &link_wrapper);
    void UpdateMenu();
    void MenuCallback(LinkMarkerWrapper &link_wrapper,
                      visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    void CreatePoseControls();
    void EnablePoseControls(bool enabled);
    void PoseCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    void GetManipulators(OpenRAVE::KinBody::LinkPtr link,
                         std::vector<OpenRAVE::RobotBase::ManipulatorPtr> *manipulators) const;
};

}

#endif
