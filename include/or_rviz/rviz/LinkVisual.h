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
#ifndef LINKVISUAL_H_
#define LINKVISUAL_H_

#include <rviz/properties/property.h>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
    #include <openrave/kinbody.h>
#endif
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

            virtual void CreateProperties(rviz::Property *parent);

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
