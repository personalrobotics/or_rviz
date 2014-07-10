#include <boost/format.hpp>
#include "ManipulatorMarker.h"
#include "or_conversions.h"

using boost::format;
using boost::str;
using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef OpenRAVE::RobotBase::ManipulatorPtr ManipulatorPtr;

// TODO: Don't hardcode this.
static std::string const kWorldFrameId = "/world";

namespace or_interactivemarker {

ManipulatorMarker::ManipulatorMarker(InteractiveMarkerServerPtr server,
                                     ManipulatorPtr manipulator)
    : server_(server)
    , manipulator_(manipulator)
    , current_pose_(manipulator->GetEndEffectorTransform())
    , changed_pose_(true)
{
    BOOST_ASSERT(server_);
    BOOST_ASSERT(manipulator);

    ik_marker_.header.frame_id = kWorldFrameId;
    ik_marker_.name = id();
    ik_marker_.description = manipulator_->GetName();
    ik_marker_.pose = toROSPose(manipulator->GetEndEffectorTransform());
    ik_marker_.scale = 0.25;

    // Create a 6-DOF pose control.
    {
        InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        ik_marker_.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        ik_marker_.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        ik_marker_.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        ik_marker_.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        ik_marker_.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        ik_marker_.controls.push_back(control);
    }

    server_->insert(ik_marker_);
    server_->setCallback(ik_marker_.name,
        boost::bind(&ManipulatorMarker::IkFeedback, this, _1));
}

std::string ManipulatorMarker::id() const
{
    OpenRAVE::RobotBasePtr const robot = manipulator_->GetRobot();
    OpenRAVE::EnvironmentBasePtr const env = robot->GetEnv();
    int const environment_id = OpenRAVE::RaveGetEnvironmentId(env);

    return str(format("Environment[%d].KinBody[%s].Manipulator[%s]")
               % environment_id % robot->GetName() % manipulator_->GetName());
}

void ManipulatorMarker::EnvironmentSync()
{
    if (changed_pose_ && manipulator_->GetIkSolver()) {
        OpenRAVE::IkParameterization ik_param;
        ik_param.SetTransform6D(current_pose_);
    
        bool const solution = manipulator_->FindIKSolution(ik_param, current_ik_, 0);
        std::cout << "Found IK? " << solution << std::endl;
        if (solution) {
            auto const dof_indices = manipulator_->GetArmIndices();
            manipulator_->GetRobot()->SetDOFValues(current_ik_, 1, dof_indices);
        }
    }

    changed_pose_ = false;
}

void ManipulatorMarker::IkFeedback(InteractiveMarkerFeedbackConstPtr const &feedback)
{
    std::cout << "Moved!" << std::endl;
    if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) {
        current_pose_ = toORPose(feedback->pose);
        changed_pose_ = true;
    }
}

}
