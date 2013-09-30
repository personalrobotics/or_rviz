/*
 * LinkVisual.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#include "LinkVisual.h"

#include "../Converters.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/object.h>


namespace superviewer
{

    LinkVisual::LinkVisual(KinBodyVisual* kinBody, OpenRAVE::KinBody::LinkPtr link, Ogre::SceneNode* parent, Ogre::SceneManager* sceneManager) :
            m_kinBody(kinBody), m_link(link), m_parentNode(parent), m_sceneManager(sceneManager)
    {
        m_sceneNode = m_parentNode->createChildSceneNode();
    }

    LinkVisual::~LinkVisual()
    {
        m_sceneManager->destroySceneNode(m_sceneNode);
    }

    void LinkVisual::CreateParts()
    {
        std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries = m_link->GetGeometries();


        for(size_t i = 0; i < geometries.size(); i++)
        {
            OpenRAVE::KinBody::Link::GeometryPtr geom = geometries.at(i);
            switch(geom->GetType())
            {
                case OpenRAVE::GT_Box:
                {
                    Ogre::Entity* boxEntity = rviz::Shape::createEntity("Box" + m_link->GetName(), rviz::Shape::Cube, m_sceneManager);
                    Ogre::SceneNode* childNode = m_sceneNode->createChildSceneNode("Box" + m_link->GetName(), converters::ToOgreVector(geom->GetTransform().trans), converters::ToOgreQuaternion(geom->GetTransform().rot));

                    break;
                }
                case OpenRAVE::GT_Cylinder:
                {
                    break;
                }
                case OpenRAVE::GT_Sphere:
                {
                    break;
                }
                case OpenRAVE::GT_TriMesh:
                {
                    break;
                }
            }
        }
    }

} /* namespace superviewer */
