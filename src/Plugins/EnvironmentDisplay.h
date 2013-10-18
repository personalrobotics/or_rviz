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

namespace or_rviz
{
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

            std::string GetFrame() { return m_frame; }
            void SetFrame(const std::string& value) { m_frame = value; fixedFrameChanged(); }

            Ogre::SceneNode* GetNode() { return m_sceneNode; }

            void RemoveKinBody(const std::string& name);

            virtual void onInitialize();
            virtual void fixedFrameChanged();
            virtual void reset();
            virtual void createProperties();

        protected:
            virtual void onEnable();
            virtual void onDisable();


            OpenRAVE::EnvironmentBaseWeakPtr m_env;
            std::map<std::string, KinBodyVisual*> m_bodyVisuals;
            Ogre::SceneManager* m_sceneManager;
            Ogre::SceneNode* m_sceneNode;

            std::string m_frame;
            rviz::TFFramePropertyWPtr m_frameProperty;
            rviz::CategoryPropertyWPtr m_kinbodiesCategory;




    };

} /* namespace superviewer */
#endif /* ENVIRONMENTDISPLAY_H_ */
