/*
 * KinBodyDisplay.h
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#ifndef KINBODYDISPLAY_H_
#define KINBODYDISPLAY_H_

#include <openrave-core.h>

#include <rviz/display.h>
#include "KinBodyVisual.h"

namespace or_rviz
{


    class KinBodyDisplay : public rviz::Display
    {
        public:
            KinBodyDisplay();
            KinBodyDisplay(OpenRAVE::KinBodyPtr kinBody, Ogre::SceneManager* sceneManager);
            virtual ~KinBodyDisplay();

            void CreateVisual(OpenRAVE::KinBodyPtr kinBody, Ogre::SceneManager* sceneManager);
            inline void UpdateTransforms() { m_visual->UpdateTransforms(); }

            virtual void onInitialize();
            virtual void fixedFrameChanged();
            virtual void reset();
            virtual void createProperties();


        protected:
            virtual void onEnable();
            virtual void onDisable();
            KinBodyVisual* m_visual;

    };

} /* namespace superviewer */
#endif /* KINBODYDISPLAY_H_ */
