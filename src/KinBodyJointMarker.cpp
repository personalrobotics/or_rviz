#include "KinBodyJointMarker.h"

using interactive_markers::InteractiveMarkerServer;
using OpenRAVE::KinBody;
using OpenRAVE::KinBodyPtr;

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;

namespace or_interactivemarker {

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

    dof_values[0] += delta();
    kinbody->SetDOFValues(dof_values, KinBody::CLA_CheckLimitsSilent, dof_indices);
    reset_delta();

    // Update the pose of the joint from OpenRAVE.
    set_pose(GetJointPose(joint));

    return JointMarker::EnvironmentSync();
}

}
