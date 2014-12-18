#ifndef MANIPULATORMARKER_H_
#define MANIPULATORMARKER_H_
#include <boost/unordered_map.hpp>
#include <openrave/openrave.h>
#include <interactive_markers/interactive_marker_server.h>
#include "LinkMarker.h"
#include "JointMarker.h"

namespace or_interactivemarker
{

    class ManipulatorMarker;
    typedef boost::shared_ptr<ManipulatorMarker> ManipulatorMarkerPtr;

    class ManipulatorMarker
    {
        public:
            static OpenRAVE::Vector const kValidColor;
            static OpenRAVE::Vector const kInvalidColor;

            ManipulatorMarker(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, OpenRAVE::RobotBase::ManipulatorPtr manipulator);
            virtual ~ManipulatorMarker();

            std::string id() const;
            bool EnvironmentSync();
            void UpdateMenu();

        private:
            boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
            OpenRAVE::RobotBase::ManipulatorPtr manipulator_;

            visualization_msgs::InteractiveMarker ik_marker_;
            visualization_msgs::InteractiveMarkerControl *ik_control_;
            boost::unordered_map<OpenRAVE::KinBody::Link *, LinkMarkerPtr> link_markers_;
            boost::unordered_map<OpenRAVE::KinBody::Joint *, JointMarkerPtr> free_joint_markers_;

            bool changed_pose_;
            bool has_ik_;
            OpenRAVE::Transform current_pose_;
            std::vector<OpenRAVE::dReal> current_ik_;
            std::vector<OpenRAVE::dReal> current_free_;

            interactive_markers::MenuHandler menu_handler_;
            interactive_markers::MenuHandler::EntryHandle menu_set_;
            interactive_markers::MenuHandler::EntryHandle menu_reset_;

            void CreateGeometry();

            void CreateMenu();
            void UpdateMenu(LinkMarkerPtr link_marker);
            void MenuCallback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);

            void IkFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr const &feedback);
            void InferFreeJoints(std::vector<OpenRAVE::KinBody::JointPtr> *free_joints) const;
    };

}

#endif
