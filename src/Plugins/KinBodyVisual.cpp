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

namespace or_rviz
{

    KinBodyVisual::KinBodyVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, OpenRAVE::KinBodyPtr kinBody) :
            m_kinBody(kinBody), m_sceneManager(sceneManager), m_parentNode(parentNode)
    {
        m_sceneNode = m_parentNode->createChildSceneNode();
        m_sceneNode->setPosition(GetKinBody()->GetTransform().trans.x, GetKinBody()->GetTransform().trans.y, GetKinBody()->GetTransform().trans.z);
        m_sceneNode->setOrientation(converters::ToOgreQuaternion(kinBody->GetTransform().rot));
        CreateParts();
        m_renderMode = LinkVisual::VisualMesh;
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


    void KinBodyVisual::UpdateTransforms()
    {
        m_sceneNode->setPosition(converters::ToOgreVector(GetKinBody()->GetTransform().trans));
        m_sceneNode->setOrientation(converters::ToOgreQuaternion(GetKinBody()->GetTransform().rot));
        for(size_t i = 0; i < GetKinBody()->GetLinks().size(); i++)
        {
            LinkVisual* visual = m_links.at(i);
            OpenRAVE::KinBody::LinkPtr link = GetKinBody()->GetLinks().at(i);

            OpenRAVE::Transform relativeTransform = GetKinBody()->GetTransform().inverse() * link->GetTransform();

            visual->GetSceneNode()->setPosition(converters::ToOgreVector(relativeTransform.trans));
            visual->GetSceneNode()->setOrientation(converters::ToOgreQuaternion(relativeTransform.rot));
        }
    }

    void KinBodyVisual::CreateParts()
    {
        for(size_t i = 0; i < GetKinBody()->GetLinks().size(); i++)
        {
            LinkVisual* linkVisual = new LinkVisual(this, GetKinBody()->GetLinks().at(i), m_sceneNode, m_sceneManager);
            m_links.push_back(linkVisual);
        }
    }

    void KinBodyVisual::SetRenderMode(LinkVisual::RenderMode mode)
    {
        m_renderMode = mode;

        for(size_t i = 0; i < m_links.size(); i++)
        {
            m_links[i]->SetRenderMode(mode);
        }
    }

} /* namespace superviewer */
