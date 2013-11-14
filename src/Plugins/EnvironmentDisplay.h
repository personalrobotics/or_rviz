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




namespace or_rviz
{



    struct ControlHandle
    {
            std::string name;
            bool createPose;
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

            void CreateControls(KinBodyVisual* visual, bool poseControl, bool immediate);
            void CreateRvizPropertyMenu(KinBodyVisual* visual);

            inline interactive_markers::MenuHandler& GetMenu(std::string name) { return m_menus[name]; }

            // RVIZ callbacks.
            virtual void onInitialize();
            virtual void fixedFrameChanged();
            virtual void reset();
            virtual void createProperties();

        protected:
            // RVIZ callbacks
            virtual void onEnable();
            virtual void onDisable();

            // Callbacks for user input events.
            void OnKinbodyMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuDelete(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuVisibleChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuMoveChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
            void OnKinbodyMenuCollisionChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

            // These exist for events which *can't* occur in a thread
            // different from the GUI thread, due to openGL context.
            void HandleControlBufferEvents();
            void HandleBodyUpdateEvents();
            void RemoveDeadBodies(std::vector<OpenRAVE::KinBodyPtr>& bodies);

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



    };

} /* namespace superviewer */
#endif /* ENVIRONMENTDISPLAY_H_ */
