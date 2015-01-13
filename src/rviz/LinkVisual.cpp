/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Matthew Klingensmith <mklingen@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#include <boost/make_shared.hpp>
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
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreMeshSerializer.h>
#include <boost/filesystem.hpp>
#include "rviz/Converters.h"
#include "rviz/LinkVisual.h"
#include "rviz/KinBodyVisual.h"

namespace or_rviz
{

void destroyAllAttachedMovableObjects(Ogre::SceneNode* node)
{
    if (!node)
        return;

    // Destroy all the attached objects
    Ogre::SceneNode::ObjectIterator itObject = node->getAttachedObjectIterator();

    while (itObject.hasMoreElements())
    {
        node->getCreator()->destroyMovableObject(itObject.getNext());
    }

    // Recurse to child SceneNodes
    Ogre::SceneNode::ChildNodeIterator itChild = node->getChildIterator();

    while (itChild.hasMoreElements())
    {
        Ogre::SceneNode* pChildNode = static_cast<Ogre::SceneNode*>(itChild.getNext());
        destroyAllAttachedMovableObjects(pChildNode);
    }
}


LinkVisual::LinkVisual(KinBodyVisual* kinBody, OpenRAVE::KinBody::LinkPtr link, Ogre::SceneNode* parent, Ogre::SceneManager* sceneManager) :
        m_kinBody(kinBody), m_link(link), m_parentNode(parent), m_sceneManager(sceneManager)
{
    m_renderMode = VisualMesh;
    m_sceneNode = m_parentNode->createChildSceneNode();
    CreateParts();
}

LinkVisual::~LinkVisual()
{
    m_sceneManager->destroySceneNode(m_sceneNode);
}

void LinkVisual::CreateProperties(rviz::Property *parent)
{
}

void DeleteRepeatedVertices(const OpenRAVE::TriMesh& trimesh, std::vector<Ogre::Vector3>& verts, std::vector<int>& indices, bool remove)
{
    for (size_t i = 0; i < trimesh.indices.size(); i++)
    {
        indices.push_back(trimesh.indices.at(i));
    }

    float epsilon = 0.0001;
    for (size_t i = 0; i < trimesh.vertices.size(); i++)
    {
        Ogre::Vector3 vec = converters::ToOgreVector(trimesh.vertices.at(i));

        bool copyFound = false;

        if (remove)
        {
            for (size_t j = 0; j < verts.size(); j++)
            {
                if (fabs(vec.x - verts[j].x) < epsilon && fabs(vec.y - verts[j].y) < epsilon && fabs(vec.z - verts[j].z) < epsilon)
                {
                    copyFound = true;
                    indices[i] = j;
                    break;
                }
            }
        }

        if (copyFound)
        {
            continue;
        }

        if (remove)
        {
            indices[i] = (int) (((verts.size())));
        }
        verts.push_back(vec);
    }
    
}

Ogre::MeshPtr LinkVisual::meshToOgre(const OpenRAVE::TriMesh& trimesh, std::string name)
{
    Ogre::MeshPtr existingMesh = Ogre::ResourceGroupManager::getSingleton()._getResourceManager("Mesh")->getByName(name, "General");
    if (existingMesh.get())
    {
        RAVELOG_DEBUG("Mesh %s exists. returning.\n", name.c_str());
        return existingMesh;
    }
    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, "General");
    Ogre::SubMesh* subMesh = mesh->createSubMesh();
    std::vector < Ogre::Vector3 > verts;
    std::vector<int> index;
    DeleteRepeatedVertices(trimesh, verts, index, false);
    std::vector < Ogre::Vector3 > normals(verts.size(), Ogre::Vector3(0, 0, 0));
    for (std::vector<int>::const_iterator i = index.begin(); i != index.end(); std::advance(i, 3))
    {
        Ogre::Vector3 v[3] =
        { (verts.at(*i)), (verts.at(*(i + 1))), ((verts.at(*(i + 2)))) };

        Ogre::Vector3 normal = (v[1] - v[0]).crossProduct(v[2] - v[0]);
        for (int j = 0; j < 3; ++j)
        {
            Ogre::Vector3 a = v[(j + 1) % 3] - v[j];
            Ogre::Vector3 b = v[(j + 2) % 3] - v[j];
            float weight = acos(a.dotProduct(b) / (a.length() * b.length()));
            normals[*(i + j)] += weight * normal;
        }
    }
    
    for (size_t i = 0; i < normals.size(); i++)
    {
        normals.at(i).normalise();
    }

    /* create the vertex data structure */
    mesh->sharedVertexData = new Ogre::VertexData;
    mesh->sharedVertexData->vertexCount = verts.size();

    /* declare how the vertices will be represented */
    Ogre::VertexDeclaration* decl = mesh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;

    /* the first three floats of each vertex represent the position */
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    /* create the vertex buffer */
    Ogre::HardwareVertexBufferSharedPtr vertexBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(offset, mesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC);

    /* lock the buffer so we can get exclusive access to its data */
    float* vertices = static_cast<float*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));
    Ogre::Vector3 min(9999, 9999, 9999);
    Ogre::Vector3 max(-9999, -9999, -9999);
    size_t i = 0;
    for (std::vector<Ogre::Vector3>::const_iterator it = verts.begin(); it != verts.end(); it++)
    {
        vertices[i * 6 + 0] = it->x;
        vertices[i * 6 + 1] = it->y;
        vertices[i * 6 + 2] = it->z;
        vertices[i * 6 + 3] = normals[i].x;
        vertices[i * 6 + 4] = normals[i].y;
        vertices[i * 6 + 5] = normals[i].z;
        min.x = fmin(it->x, min.x);
        min.y = fmin(it->y, min.y);
        min.z = fmin(it->z, min.z);
        max.x = fmax(it->x, max.x);
        max.y = fmax(it->y, max.y);
        max.z = fmax(it->z, max.z);
        i++;
    }

    /* unlock the buffer */
    vertexBuffer->unlock();

    /* create the index buffer */
    Ogre::HardwareIndexBufferSharedPtr indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, trimesh.indices.size(), Ogre::HardwareBuffer::HBU_STATIC);

    /* lock the buffer so we can get exclusive access to its data */
    uint16_t* indices = static_cast<uint16_t*>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));
    i = 0;
    for (std::vector<int>::const_iterator it = index.begin(); it != index.end(); it++)
    {
        indices[i] = static_cast<uint16_t>(*it);
        i++;
    }

    /* unlock the buffer */
    indexBuffer->unlock();

    /* attach the buffers to the mesh */
    mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vertexBuffer);
    subMesh->useSharedVertices = true;
    subMesh->indexData->indexBuffer = indexBuffer;
    subMesh->indexData->indexCount = index.size();
    subMesh->indexData->indexStart = 0;

    /* set the bounds of the mesh */
    mesh->_setBounds(Ogre::AxisAlignedBox(min.x, min.y, min.z, max.x, max.y, max.z));

    /* notify the mesh that we're all ready */
    mesh->load();
    return mesh;
}

void LinkVisual::LoadRenderMesh(std::string& fileName, OpenRAVE::KinBody::Link::GeometryPtr& geom, Ogre::Entity*& entity, std::string& objectName, std::stringstream& idString, Ogre::Vector3& scale)
{
    //
    Ogre::MeshPtr mesh;
    try
    {
        mesh = rviz::loadMeshFromResource("file://" + fileName);
    }
    catch (Ogre::Exception& e)
    {
        //RAVELOG_DEBUG("Mesh %s can't be loaded by STL or .mesh loader. %s Falling back to OpenRAVE geometry.\n", fileName.c_str(), e.what());
    }

    if (!mesh.get())
    {
        boost::shared_ptr<OpenRAVE::TriMesh> myMesh = boost::make_shared<OpenRAVE::TriMesh>();
        m_kinBody->GetKinBody()->GetEnv()->ReadTrimeshFile(myMesh, geom->GetRenderFilename());

        try {
            if (myMesh->vertices.size() >= 3) {
                std::string const meshName = getMeshName(fileName);
                mesh = meshToOgre(*myMesh, meshName);
            }
        } catch (Ogre::InternalErrorException const &ex) {
            RAVELOG_ERROR(ex.what());
        }
    }
    else
    {
        //RAVELOG_DEBUG("Successfully loaded mesh: %s\n", fileName.c_str());
    }

    entity = m_sceneManager->createEntity("Mesh " + objectName + idString.str(), mesh->getName(), mesh->getGroup());
    entity->setVisible(true);
    scale = converters::ToOgreVector(geom->GetRenderScale());
}

std::string LinkVisual::getMeshName(std::string const filename) const
{
    static int id = 0;

    if (!filename.empty()) {
        return filename;
    } else {
        std::stringstream ss;
        ss << "mesh" << id;
        id++;
        return ss.str();
    }
}

void LinkVisual::CreateCollisionGeometry(OpenRAVE::KinBody::Link::GeometryPtr& geom,
    Ogre::Entity*& entity, std::string& objectName, std::stringstream& idString,
    Ogre::Vector3& scale, Ogre::Quaternion& offset_orientation, std::string fileName)
{
    switch (geom->GetType())
    {
        case OpenRAVE::GT_Box:
        {
            entity = rviz::Shape::createEntity("Box" + objectName + idString.str(), rviz::Shape::Cube, m_sceneManager);
            scale = converters::ToOgreVector(geom->GetBoxExtents() * 2);
            break;
        }
        case OpenRAVE::GT_Cylinder:
        {
            Ogre::Quaternion rotX;
            rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
            offset_orientation = offset_orientation * rotX;
            
            entity = rviz::Shape::createEntity("Cylinder" + objectName + idString.str(), rviz::Shape::Cylinder, m_sceneManager);
            scale = Ogre::Vector3(geom->GetCylinderRadius() * 2, geom->GetCylinderHeight(), geom->GetCylinderRadius() * 2);
            break;
        }
        case OpenRAVE::GT_Sphere:
        {
            entity = rviz::Shape::createEntity("Sphere" + objectName + idString.str(), rviz::Shape::Sphere, m_sceneManager);
            scale = Ogre::Vector3(geom->GetSphereRadius() * 2, geom->GetSphereRadius() * 2, geom->GetSphereRadius() * 2);
            break;
        }
        case OpenRAVE::GT_TriMesh:
        {
            Ogre::MeshPtr mesh;
            const OpenRAVE::TriMesh& myMesh = geom->GetCollisionMesh();
            
            try
            {
                if (myMesh.vertices.size() >= 3) {
                    std::string const meshName = getMeshName(fileName);
                    mesh = meshToOgre(myMesh, meshName);
                }
            }
            catch (Ogre::InternalErrorException& ex)
            {
                RAVELOG_ERROR(ex.what());
            }
            
            if (mesh.get())
            {
                entity = m_sceneManager->createEntity("Mesh " + objectName + idString.str(), mesh->getName(), mesh->getGroup());
                entity->setVisible(true);
                scale = converters::ToOgreVector(geom->GetRenderScale());
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
}

void LinkVisual::CreateMaterial(std::string& objectName, std::stringstream& idString, OpenRAVE::KinBody::Link::GeometryPtr geom)
{
    Ogre::MaterialManager& matMgr = Ogre::MaterialManager::getSingleton();
    Ogre::ResourceManager::ResourceCreateOrRetrieveResult result = matMgr.createOrRetrieve(objectName + " Material" + idString.str(), Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
    if (result.second)
    {
        Ogre::MaterialPtr mat = (Ogre::MaterialPtr)(result.first);
        mat->setReceiveShadows(true);

        Ogre::Technique* technique = NULL;

        if(mat->getNumTechniques() == 0)
        {
            technique = mat->createTechnique();
        }
        else
        {
            technique = mat->getTechnique(0);
        }

        Ogre::Pass* pass = NULL;

        if(technique->getNumPasses() == 0)
        {
            pass = technique->createPass();
        }
        else
        {
            pass = technique->getPass(0);
        }

        pass->setAmbient(geom->GetAmbientColor().x, geom->GetAmbientColor().y, geom->GetAmbientColor().z);
        pass->setDiffuse(geom->GetDiffuseColor().x, geom->GetDiffuseColor().y, geom->GetDiffuseColor().z, 1.0f - geom->GetTransparency());

        if(geom->GetTransparency() > 0.01f)
        {
            pass->setDepthCheckEnabled(true);
            pass->setDepthWriteEnabled(false);
            pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        }

        mat->compile();
    }
}

void LinkVisual::CreateGeometry(OpenRAVE::KinBody::Link::GeometryPtr geom)
{
    static int id = 0;

    if (m_renderMode == LinkVisual::VisualMesh && !geom->IsVisible())
    {
        return;
    }
    // TODO: This might be wrong.
    else if(m_renderMode == LinkVisual::CollisionMesh && geom->IsVisible())
    {
        return;
    }

    std::stringstream idString;
    idString << id;
    id++;

    Ogre::SceneNode* offsetNode = m_sceneNode->createChildSceneNode();
    Ogre::Entity* entity = NULL;
    Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);
    Ogre::Vector3 offset_position = converters::ToOgreVector(geom->GetTransform().trans);
    Ogre::Quaternion offset_orientation = converters::ToOgreQuaternion(geom->GetTransform().rot);

    std::string objectName = GetLink()->GetName() + " " + m_kinBody->GetKinBody()->GetName();
    std::string fileName = m_renderMode == CollisionMesh ? geom->GetInfo()._filenamecollision : geom->GetInfo()._filenamerender;

    // If there is a render mesh we will ignore all of the other geometry.
    if (!fileName.empty())
    {
        LoadRenderMesh(fileName, geom, entity, objectName, idString, scale);
    }
    else
    {
        CreateCollisionGeometry(geom, entity, objectName, idString, scale, offset_orientation, fileName);
    }

    if (entity)
    {
        CreateMaterial(objectName, idString, geom);

        offsetNode->attachObject(entity);
        offsetNode->setScale(scale);
        offsetNode->setPosition(offset_position);
        offsetNode->setOrientation(offset_orientation);

        for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
        {
            Ogre::SubEntity* sub = entity->getSubEntity(i);
            if(sub->getMaterialName() == "BaseWhiteNoLighting")
            {
                sub->setMaterialName(objectName + " Material" + idString.str());
            }
            else
            {
                Ogre::MaterialPtr material = sub->getMaterial();
                Ogre::Technique* technique = material->getTechnique(0);
                //technique->setLightingEnabled(false);
            }
        }

    }
}

void LinkVisual::CreateParts()
{

    destroyAllAttachedMovableObjects(m_sceneNode);
    m_sceneNode->removeAllChildren();

    std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries = GetLink()->GetGeometries();

    for (size_t i = 0; i < geometries.size(); i++)
    {

        OpenRAVE::KinBody::Link::GeometryPtr geom = geometries.at(i);
        CreateGeometry(geom);
    }
}

} /* namespace superviewer */
