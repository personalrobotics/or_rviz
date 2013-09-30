/*
 * KinBodyVisual.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#include "KinBodyVisual.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMesh.h>

namespace superviewer
{

    KinBodyVisual::KinBodyVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, OpenRAVE::KinBodyPtr kinBody) :
            m_kinBody(kinBody), m_sceneManager(sceneManager), m_parentNode(parentNode)
    {
        m_sceneNode = m_parentNode->createChildSceneNode();

    }

    KinBodyVisual::~KinBodyVisual()
    {
        m_sceneManager->destroySceneNode(m_sceneNode);
    }


    void KinBodyVisual::CreateParts()
    {

    }

} /* namespace superviewer */
