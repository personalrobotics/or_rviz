#ifndef KINBODYLINKMARKER_H_
#define KINBODYLINKMARKER_H_
#include <interactive_markers/interactive_marker_server.h>
#include "LinkMarker.h"

namespace or_interactivemarker {

class KinBodyLinkMarker;
typedef boost::shared_ptr<KinBodyLinkMarker> KinBodyLinkMarkerPtr;

class KinBodyLinkMarker : public LinkMarker {
public:
    KinBodyLinkMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                      OpenRAVE::KinBody::LinkPtr link);

    interactive_markers::MenuHandler &menu_handler();

    virtual bool EnvironmentSync();
    void UpdateMenu();

private:
    typedef interactive_markers::MenuHandler MenuHandler;

    bool menu_changed_;
    std::vector<visualization_msgs::MenuEntry> menu_entries_;
    MenuHandler menu_handler_;
    MenuHandler::EntryHandle menu_link_;
    MenuHandler::EntryHandle menu_visible_;
    MenuHandler::EntryHandle menu_enabled_;
    MenuHandler::EntryHandle menu_geom_;
    MenuHandler::EntryHandle menu_geom_visual_;
    MenuHandler::EntryHandle menu_geom_collision_;
    MenuHandler::EntryHandle menu_geom_both_;
    MenuHandler::EntryHandle menu_groups_;
    boost::unordered_map<std::string, MenuHandler::EntryHandle> menu_groups_entries_;

    void CreateMenu();
    void MenuCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
};


}

#endif
