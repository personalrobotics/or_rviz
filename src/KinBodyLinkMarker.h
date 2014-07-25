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
    bool menu_changed_;
    std::vector<visualization_msgs::MenuEntry> menu_entries_;
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_link_;
    interactive_markers::MenuHandler::EntryHandle menu_visible_;
    interactive_markers::MenuHandler::EntryHandle menu_enabled_;
    interactive_markers::MenuHandler::EntryHandle menu_geom_;
    interactive_markers::MenuHandler::EntryHandle menu_geom_visual_;
    interactive_markers::MenuHandler::EntryHandle menu_geom_collision_;
    interactive_markers::MenuHandler::EntryHandle menu_geom_both_;

    void CreateMenu();
    void MenuCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
};


}

#endif
