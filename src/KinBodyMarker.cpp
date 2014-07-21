#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "KinBodyMarker.h"

using boost::format;
using boost::str;
using boost::algorithm::ends_with;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::KinBodyWeakPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBaseWeakPtr;
using interactive_markers::InteractiveMarkerServer;

typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;
typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

namespace or_interactivemarker {

KinBodyMarker::KinBodyMarker(InteractiveMarkerServerPtr server,
                             KinBodyPtr kinbody)
    : server_(server)
    , kinbody_(kinbody)
    , robot_(boost::dynamic_pointer_cast<RobotBase>(kinbody))
{
    BOOST_ASSERT(server);
    BOOST_ASSERT(kinbody);

#if 0
    if (!IsGhost()) {
        CreateGhost();
    }
#endif
}

KinBodyMarker::~KinBodyMarker()
{
    if (ghost_kinbody_) {
        ghost_kinbody_->GetEnv()->Remove(ghost_kinbody_);
        ghost_kinbody_.reset();
        ghost_robot_.reset();
    }
}

bool KinBodyMarker::IsGhost() const
{
    KinBodyPtr kinbody = kinbody_.lock();
    return ends_with(kinbody->GetName(), ".Ghost");
}

void KinBodyMarker::EnvironmentSync()
{
    typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
    typedef OpenRAVE::KinBody::JointPtr JointPtr;

    KinBodyPtr const kinbody = kinbody_.lock();
    bool const is_ghost = IsGhost();

    // Update links. This includes the geometry of the KinBody.
    for (LinkPtr link : kinbody->GetLinks()) {
        LinkMarkerPtr &link_marker = link_markers_[link.get()];
        if (!link_marker) {
            link_marker = boost::make_shared<LinkMarker>(server_, link, is_ghost);
        }
        link_marker->EnvironmentSync();
    }

#if 0
    // Update joints.
    for (JointPtr joint : kinbody_->GetJoints()) {
        JointMarkerPtr &joint_marker = joint_markers_[joint.get()];
        if (!joint_marker) {
            joint_marker = boost::make_shared<JointMarker>(server_, joint);
        }
        joint_marker->EnvironmentSync();
    }

    // Also update manipulators if we're a robot.
    if (robot_ && !ManipulatorMarker::IsGhost(kinbody_)) {
        for (ManipulatorPtr const manipulator : robot_->GetManipulators()) {
            auto const it = manipulator_markers_.find(manipulator.get());
            BOOST_ASSERT(it != manipulator_markers_.end());
            it->second->EnvironmentSync();
        }
    }
#endif
}

void KinBodyMarker::CreateGhost()
{
    KinBodyPtr kinbody = kinbody_.lock();

    EnvironmentBasePtr env = kinbody->GetEnv();
    if (kinbody->IsRobot()) {
        ghost_robot_ = OpenRAVE::RaveCreateRobot(env, "");
        ghost_kinbody_ = ghost_robot_;
    } else {
        ghost_kinbody_ = OpenRAVE::RaveCreateKinBody(env, "");
    }

    ghost_robot_->Clone(kinbody, OpenRAVE::Clone_Bodies);
    ghost_robot_->SetName(kinbody->GetName() + ".Ghost");
    ghost_robot_->Enable(false);
    env->Add(ghost_kinbody_, true);
}

}
