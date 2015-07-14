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
#ifndef MANIPULATORMARKER_H_
#define MANIPULATORMARKER_H_
#include <boost/unordered_map.hpp>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif
#include <interactive_markers/interactive_marker_server.h>
#include "LinkMarker.h"
#include "JointMarker.h"

namespace or_rviz {
namespace markers {

class ManipulatorMarker;
typedef boost::shared_ptr<ManipulatorMarker> ManipulatorMarkerPtr;

class ManipulatorMarker {
public:
    static OpenRAVE::Vector const kValidColor;
    static OpenRAVE::Vector const kInvalidColor;

    ManipulatorMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                      OpenRAVE::RobotBase::ManipulatorPtr manipulator);
    virtual ~ManipulatorMarker();

    std::string id() const;
    bool is_hidden() const;

    void set_parent_frame(std::string const &frame_id);

    bool EnvironmentSync();
    void UpdateMenu();

private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    OpenRAVE::RobotBase::ManipulatorPtr manipulator_;

    visualization_msgs::InteractiveMarker ik_marker_;
    visualization_msgs::InteractiveMarkerControl *ik_control_;
    boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerPtr> link_markers_;
    boost::unordered_map<OpenRAVE::KinBody::Joint *, JointMarkerPtr> free_joint_markers_;

    bool reset_pose_;
    bool changed_pose_;
    bool has_ik_;
    bool force_update_;
    bool hidden_;

    OpenRAVE::Transform current_pose_;
    std::vector<OpenRAVE::dReal> current_ik_;
    std::vector<OpenRAVE::dReal> current_free_;

    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_set_;
    interactive_markers::MenuHandler::EntryHandle menu_reset_;
    interactive_markers::MenuHandler::EntryHandle menu_hide_;

    void CreateGeometry();

    void CreateMenu();
    void UpdateMenu(LinkMarkerPtr link_marker);
    void MenuCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

    void IkFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
    void InferFreeJoints(std::vector<OpenRAVE::KinBody::JointPtr> *free_joints) const;
};

}
}

#endif
