/*
 * LinkVisual.h
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#ifndef LINKVISUAL_H_
#define LINKVISUAL_H_

#include <openrave/openrave.h>
#include <openrave/kinbody.h>
#include <vector>
#include "Ogre.h"

namespace or_rviz
{
    class KinBodyVisual;
    class LinkVisual
    {
        public:

            enum RenderMode
            {
                CollisionMesh,
                VisualMesh
            };

            LinkVisual(KinBodyVisual* kinBody, OpenRAVE::KinBody::LinkPtr link, Ogre::SceneNode* parent, Ogre::SceneManager* sceneManager);
            virtual ~LinkVisual();

            inline OpenRAVE::KinBody::LinkPtr GetLink() { return m_link.lock(); }
            inline void SetLink(OpenRAVE::KinBody::LinkPtr value) { m_link = value; }

            inline Ogre::SceneNode* GetSceneNode() { return m_sceneNode; }
            inline void SetSceneNode(Ogre::SceneNode* value) { m_sceneNode = value; }

            inline KinBodyVisual* GetKinBody() { return m_kinBody; }
            inline void SetKinBody(KinBodyVisual* value) { m_kinBody = value; }
            Ogre::MeshPtr meshToOgre(const OpenRAVE::TriMesh& trimesh, std::string name);
            std::string getMeshName(std::string const filename) const;


            inline void SetRenderMode(RenderMode mode) { m_renderMode = mode; CreateParts(); }
            inline RenderMode GetRenderMode() { return m_renderMode; }

            void CreateParts();

        protected:

            void CreateGeometry(OpenRAVE::KinBody::Link::GeometryPtr geom);
            void CreateRenderMesh();
            void LoadRenderMesh(std::string& fileName,
			OpenRAVE::KinBody::Link::GeometryPtr& geom, Ogre::Entity*& entity,
			std::string& objectName, std::stringstream& idString,
			Ogre::Vector3& scale);
            void CreateCollisionGeometry(OpenRAVE::KinBody::Link::GeometryPtr& geom, Ogre::Entity*& entity, std::string& objectName, std::stringstream& idString, Ogre::Vector3& scale, Ogre::Quaternion& offset_orientation, std::string fileName);
            void CreateMaterial(std::string& objectName, std::stringstream& idString, OpenRAVE::KinBody::Link::GeometryPtr geom);

            KinBodyVisual* m_kinBody;
            OpenRAVE::KinBody::LinkWeakPtr m_link;
            Ogre::SceneNode* m_sceneNode;
            Ogre::SceneNode* m_parentNode;
            Ogre::SceneManager* m_sceneManager;
            RenderMode m_renderMode;

    };

} /* namespace superviewer */
#endif /* LINKVISUAL_H_ */
