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
#ifndef JOINTMARKER_H_
#define JOINTMARKER_H_
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif
#include <interactive_markers/interactive_marker_server.h>

namespace or_rviz {
namespace markers {

class JointMarker;
typedef boost::shared_ptr<JointMarker> JointMarkerPtr;

class JointMarker {
public:
    JointMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                OpenRAVE::KinBody::JointPtr joint);
    virtual ~JointMarker();

    std::string id() const;
    OpenRAVE::KinBody::JointPtr joint() const;

    void set_parent_frame(std::string const &frame_id);

    OpenRAVE::Transform pose() const;
    void set_pose(OpenRAVE::Transform const &pose);

    double angle() const;
    
    void set_joint_pose(OpenRAVE::Transform const &pose);

    virtual bool EnvironmentSync();

    static OpenRAVE::Transform GetJointPose(OpenRAVE::KinBody::JointPtr joint);

protected:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    visualization_msgs::InteractiveMarker marker_;
    visualization_msgs::InteractiveMarkerControl *joint_control_;

private:
    OpenRAVE::KinBody::JointWeakPtr joint_;
    OpenRAVE::Transform joint_pose_;
    double joint_initial_;
    double joint_delta_;
    bool created_;
    bool force_update_;
    bool active_;

    bool is_implemented_;  // does this marker do anything?

    void JointCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
};

}
}

#endif
