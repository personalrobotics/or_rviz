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
#include <Ogre.h>
#include <OgreSubEntity.h>
#include <OgreMovableObject.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/object.h>
#include <OgreException.h>
#include <OgreMeshManager.h>
#include <OgreMeshSerializer.h>
#include "KinBodyVisual.h"

namespace superviewer
{

    LinkVisual::LinkVisual(KinBodyVisual* kinBody, OpenRAVE::KinBody::LinkPtr link, Ogre::SceneNode* parent, Ogre::SceneManager* sceneManager) :
            m_kinBody(kinBody), m_link(link), m_parentNode(parent), m_sceneManager(sceneManager)
    {
        m_sceneNode = m_parentNode->createChildSceneNode();
        CreateParts();
    }

    LinkVisual::~LinkVisual()
    {
        m_sceneManager->destroySceneNode(m_sceneNode);
    }


    void LinkVisual::CreateParts()
    {
        std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries = m_link->GetGeometries();


        for (size_t i = 0; i < geometries.size(); i++)
        {
            std::stringstream ss;
            ss << i;
            Ogre::SceneNode* offsetNode = m_sceneNode->createChildSceneNode();
            OpenRAVE::KinBody::Link::GeometryPtr geom = geometries.at(i);
            Ogre::Entity* entity = NULL;
            Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);
            Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
            Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);

            {
                offset_position = converters::ToOgreVector(geom->GetTransform().trans);
                offset_orientation = converters::ToOgreQuaternion(geom->GetTransform().rot);
            }

            std::string objectName = m_link->GetName() + " " + m_kinBody->GetKinBody()->GetName();

            switch (geom->GetType())
            {
                case OpenRAVE::GT_Box:
                {
                    entity = rviz::Shape::createEntity("Box" + objectName + ss.str(), rviz::Shape::Cube, m_sceneManager);
                    scale = converters::ToOgreVector(geom->GetBoxExtents());
                    break;
                }
                case OpenRAVE::GT_Cylinder:
                {
                    Ogre::Quaternion rotX;
                    rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
                    offset_orientation = offset_orientation * rotX;

                    entity = rviz::Shape::createEntity("Cylinder" + objectName + ss.str(), rviz::Shape::Cylinder, m_sceneManager);
                    scale = Ogre::Vector3(geom->GetCylinderRadius() * 2, geom->GetCylinderHeight(), geom->GetCylinderRadius() * 2);
                    break;
                }
                case OpenRAVE::GT_Sphere:
                {
                    entity = rviz::Shape::createEntity("Sphere" + objectName + ss.str(), rviz::Shape::Sphere, m_sceneManager);
                    scale = Ogre::Vector3(geom->GetSphereRadius() * 2, geom->GetSphereRadius() * 2, geom->GetSphereRadius() * 2);
                    break;
                }
                case OpenRAVE::GT_TriMesh:
                {
                    std::string meshFile = geom->GetInfo()._filenamerender;
                    Ogre::MeshPtr ogreMesh = rviz::loadMeshFromResource(meshFile);
                    scale = converters::ToOgreVector(geom->GetRenderScale());

                    try
                    {
                        m_sceneManager->createEntity(meshFile + m_link->GetName() + ss.str(), meshFile);
                    }
                    catch (Ogre::Exception& e)
                    {
                        RAVELOG_ERROR("Could not load model '%s' for link '%s': %s\n", meshFile.c_str(), m_link->GetName().c_str(), e.what());
                    }
                    break;
                }
                default:
                {
                    RAVELOG_WARN("Unrecognized geometry type.");
                    break;
                }
            }

            if (entity)
            {
                offsetNode->attachObject(entity);
                offsetNode->setScale(scale);
                offsetNode->setPosition(offset_position);
                offsetNode->setOrientation(offset_orientation);

                for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
                {
                    // Assign materials only if the submesh does not have one already
                    Ogre::SubEntity* sub = entity->getSubEntity(i);
                    const std::string& material_name = sub->getMaterialName();

                    if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
                    {
                        sub->setMaterialName("");
                    }
                    else
                    {
                        // Need to clone here due to how selection works.  Once selection id is done per object and not per material,
                        // this can go away
                        static int count = 0;
                        std::stringstream ss;
                        ss << material_name << count++ << "Robot";
                        std::string cloned_name = ss.str();
                        sub->getMaterial()->clone(cloned_name);
                        sub->setMaterialName(cloned_name);
                    }

                    //materials_[sub] = sub->getMaterial();
                }
            }
        }
    }


} /* namespace superviewer */
