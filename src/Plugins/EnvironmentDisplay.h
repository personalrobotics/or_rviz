/*
 * EnvironmentDisplay.h
 *
 *  Created on: Oct 17, 2013
 *      Author: mklingen
 */

#ifndef ENVIRONMENTDISPLAY_H_
#define ENVIRONMENTDISPLAY_H_

#include <openrave-core.h>
#include <rviz/display.h>
#include <map>
#include "KinBodyVisual.h"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/properties/property.h>
#include <rviz/visualization_manager.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



namespace or_rviz
{



    namespace control_mode
    {
        enum ControlMode
        {
            NoControl,
            PoseControl,
            JointControl
        };
    }

    struct ControlHandle
    {
            std::string name;
            control_mode::ControlMode mode;
    };



    class EnvironmentDisplay : public rviz::Display
    {

        public:
            EnvironmentDisplay();
            virtual ~EnvironmentDisplay();

            void SetEnvironment(OpenRAVE::EnvironmentBasePtr env) { m_env = env; Clear();}
            OpenRAVE::EnvironmentBasePtr GetEnvironment() { return m_env.lock(); }

            void UpdateObjects();
            bool HasKinBody(const std::string& name) { return m_bodyVisuals.find(name) != m_bodyVisuals.end(); }
            void Clear();

            inline std::string GetFrame() { return m_frame; }
            inline void SetFrame(const std::string& value) { m_frame = value; fixedFrameChanged(); }

            inline Ogre::SceneNode* GetNode() { return m_sceneNode; }

            inline interactive_markers::InteractiveMarkerServer* GetMarkerServer() { return m_markerServer; }


            void RemoveKinBody(const std::string& name);

            void CreateControls(KinBodyVisual* visual, control_mode::ControlMode mode, bool immediate);
            void CreateRvizPropertyMenu(KinBodyVisual* visual);

            inline interactive_markers::MenuHandler& GetMenu(std::string name) { return m_menus[name]; }

            // RVIZ callbacks.
            virtual void onInitialize();
            virtual void fixedFrameChanged();
            virtual void reset();
            virtual void createProperties();

            bool RegisterMenuCallback(const std::string& objectName, const std::string& menuName, const std::string& pyObject);
            bool UnRegisterMenuCallback(const std::string& objectName, const std::string& menuName);

        protected:
            // RVIZ callbacks
            virtual void onEnable();
            virtual void onDisable();

            // Callbacks for user input events.
            void OnKinbodyMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnJointMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnCalibratorMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuDelete(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuVisibleChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuMoveChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuJointControlChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuCollisionChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void UpdateJointControlPoses(KinBodyVisual* visual);


            // These exist for events which *can't* occur in a thread
            // different from the GUI thread, due to openGL context.
            void HandleControlBufferEvents();
            void HandleBodyUpdateEvents();
            void RemoveDeadBodies(std::vector<OpenRAVE::KinBodyPtr>& bodies);
            void CreatePoseControl(visualization_msgs::InteractiveMarker& marker);
            void CreateJointControl(visualization_msgs::InteractiveMarker& marker, OpenRAVE::KinBody::JointPtr& joint);
            void CreateJointDOFControl(visualization_msgs::InteractiveMarker& marker, const OpenRAVE::Vector& axis, int jointID, int dofID, OpenRAVE::KinBody::Joint::JointType type);
            void CreateTransformController(visualization_msgs::InteractiveMarker& marker, const std::string& tfFrom, const std::string& tfTo, const std::string& fixedFrame);

            // Aligns a transform such that the z axis points in the given direction
            inline OpenRAVE::Transform ComputeFacingMatrix(OpenRAVE::Vector direction)
            {
                // x y z
                // x = y x z
                // y = z x x
                // z = x x y
                OpenRAVE::Vector zAxis = direction;
                zAxis.normalize3();

                OpenRAVE::Vector up = OpenRAVE::Vector(0, 1, 0);

                OpenRAVE::Vector yAxis = up.cross(zAxis);

                if(yAxis.lengthsqr3() < 0.001)
                {
                    up = OpenRAVE::Vector(1, 0, 0);
                    yAxis = up.cross(zAxis);
                }

                yAxis.normalize3();

                OpenRAVE::Vector xAxis = yAxis.cross(zAxis);
                xAxis.normalize3();

                OpenRAVE::Transform toReturn;
                toReturn.identity();
                OpenRAVE::TransformMatrix transformMatrix;
                transformMatrix.identity();
                transformMatrix.rot(0, 0) = xAxis.x;
                transformMatrix.rot(1, 0) = xAxis.y;
                transformMatrix.rot(2, 0) = xAxis.z;
                transformMatrix.rot(0, 1) = yAxis.x;
                transformMatrix.rot(1, 1) = yAxis.y;
                transformMatrix.rot(2, 1) = yAxis.z;
                transformMatrix.rot(0, 2) = zAxis.x;
                transformMatrix.rot(1, 2) = zAxis.y;
                transformMatrix.rot(2, 2) = zAxis.z;
                toReturn.rot = OpenRAVE::geometry::quatFromMatrix(transformMatrix);
                return toReturn;
            }

            OpenRAVE::EnvironmentBaseWeakPtr m_env;
            std::map<std::string, KinBodyVisual*> m_bodyVisuals;
            Ogre::SceneManager* m_sceneManager;
            Ogre::SceneNode* m_sceneNode;
            interactive_markers::InteractiveMarkerServer* m_markerServer;


            std::string m_frame;
            rviz::TFFramePropertyWPtr m_frameProperty;
            rviz::CategoryPropertyWPtr m_kinbodiesCategory;

            std::map<std::string, interactive_markers::MenuHandler> m_menus;
            std::vector<ControlHandle> m_controlBuffer;
            std::vector<BodyUpdateEvent> m_updateBuffer;

            std::map<std::string, std::map<std::string, std::string> > m_pythonCallbacks;

            tf::TransformListener m_tfListener;
            tf::TransformBroadcaster m_tfBroadcaster;


    };

} /* namespace superviewer */
#endif /* ENVIRONMENTDISPLAY_H_ */
