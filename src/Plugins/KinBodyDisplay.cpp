/*
 * KinBodyDisplay.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#include "../Converters.h"
#include "KinBodyDisplay.h"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace superviewer
{

    KinBodyDisplay::KinBodyDisplay(OpenRAVE::KinBodyPtr kinBod, Ogre::SceneManager* sceneManager)
    {
        m_visual = new KinBodyVisual(sceneManager, sceneManager->getRootSceneNode(), kinBod);
    }

    KinBodyDisplay::~KinBodyDisplay()
    {
        delete m_visual;
    }

    void  KinBodyDisplay::onInitialize()
    {
        //TODO: Implement
    }

    void  KinBodyDisplay::fixedFrameChanged()
    {
        //TODO: Implement
    }

    void  KinBodyDisplay::reset()
    {
        //TODO: Implement
    }

    void  KinBodyDisplay::createProperties()
    {
        //TODO: Implement
    }


} /* namespace superviewer */
