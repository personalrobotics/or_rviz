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
#include "markers/KinBodyJointMarker.h"

using interactive_markers::InteractiveMarkerServer;
using OpenRAVE::KinBody;
using OpenRAVE::KinBodyPtr;

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;

namespace or_interactivemarker {
namespace markers {

KinBodyJointMarker::KinBodyJointMarker(InteractiveMarkerServerPtr server, JointPtr joint)
    : JointMarker(server, joint)
{
}

KinBodyJointMarker::~KinBodyJointMarker()
{
}

bool KinBodyJointMarker::EnvironmentSync()
{
    // Update the KinBody in the OpenRAVE environment.
    JointPtr const joint = this->joint();
    KinBodyPtr const kinbody = joint->GetParent();
    std::vector<int> dof_indices;
    std::vector<OpenRAVE::dReal> dof_values;
    dof_indices.push_back(joint->GetJointIndex());
    kinbody->GetDOFValues(dof_values, dof_indices);
    BOOST_ASSERT(joint->GetDOF() == 1);
    BOOST_ASSERT(dof_values.size() == 1);

    dof_values[0] = angle();
    kinbody->SetDOFValues(dof_values, KinBody::CLA_CheckLimitsSilent, dof_indices);

    // Update the pose of the joint from OpenRAVE.
    set_pose(GetJointPose(joint));

    return JointMarker::EnvironmentSync();
}

}
}
