/*
 * KinBodyVisual.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#include "../Converters.h"

#include "KinBodyVisual.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMesh.h>

#include "LinkVisual.h"

namespace superviewer
{

    KinBodyVisual::KinBodyVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, OpenRAVE::KinBodyPtr kinBody) :
            m_kinBody(kinBody), m_sceneManager(sceneManager), m_parentNode(parentNode)
    {
        m_sceneNode = m_parentNode->createChildSceneNode();
        m_sceneNode->setPosition(kinBody->GetTransform().trans.x, kinBody->GetTransform().trans.y, kinBody->GetTransform().trans.z);
        m_sceneNode->setOrientation(converters::ToOgreQuaternion(kinBody->GetTransform().rot));
        CreateParts();
    }

    KinBodyVisual::~KinBodyVisual()
    {
        if(m_sceneNode)
        {
            m_sceneManager->destroySceneNode(m_sceneNode);
            m_sceneNode = NULL;
        }

        for(size_t i = 0; i < m_links.size(); i++)
        {
            delete m_links[i];
        }

    }


    void KinBodyVisual::CreateParts()
    {
        for(size_t i = 0; i < m_kinBody->GetLinks().size(); i++)
        {
            LinkVisual* linkVisual = new LinkVisual(this, m_kinBody->GetLinks().at(i), m_sceneNode, m_sceneManager);
            m_links.push_back(linkVisual);
        }
    }

} /* namespace superviewer */
