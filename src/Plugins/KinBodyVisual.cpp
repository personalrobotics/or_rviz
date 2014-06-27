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

    void KinBodyVisual::CreateProperties(rviz::Property *parent)
    {
        OpenRAVE::KinBodyPtr kinbody = GetKinBody();
        if (!kinbody) {
            return;
        }

        std::string const name = kinbody->GetName();
        OpenRAVE::Transform const pose = kinbody->GetTransform();

        m_property_parent = new rviz::Property(
            QString::fromStdString(kinbody->GetName()), QVariant(), "", parent);
        m_property_enabled = new rviz::BoolProperty("Enabled",
            true, "Enable or disable collision checking.", m_property_parent);

        m_property_visual = new rviz::EnumProperty("Render",
            "Render", "Geometry to display.", m_property_parent);
        m_property_visual->addOption("None");
        m_property_visual->addOption("Render");
        m_property_visual->addOption("Collision");
        m_property_visual->addOption("Both");

        m_property_position = new rviz::VectorProperty("Position",
            Ogre::Vector3(pose.trans.x, pose.trans.y, pose.trans.y),
            "Position in the world frame.", m_property_parent);
        m_property_orientation = new rviz::QuaternionProperty("Orientation",
            Ogre::Quaternion(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.y),
            "Orientation in the world frame.", m_property_parent);

        m_property_orientation;
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

    void KinBodyVisual::UpdateVisible()
    {
    }

} /* namespace superviewer */
