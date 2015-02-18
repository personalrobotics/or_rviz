/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#ifndef KINBODYLINKMARKER_H_
#define KINBODYLINKMARKER_H_
#include <interactive_markers/interactive_marker_server.h>
#include "LinkMarker.h"

namespace or_rviz {
namespace markers {

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
}

#endif
