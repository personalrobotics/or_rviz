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
#ifndef KINBODYVISUAL_H_
#define KINBODYVISUAL_H_

// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
    #include <openrave/kinbody.h>
#endif
#include <vector>
#include <rviz/properties/property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/bool_property.h>
#include <OgreSceneNode.h>
#include "LinkVisual.h"
#include <QObject>

namespace Ogre
{
    class SceneManager;
    class SceneNode;
}

namespace or_rviz
{

enum BodyUpdateType
{
    BodyUpdateRenderMode
};

struct BodyUpdateEvent
{
    std::string bodyName;
    BodyUpdateType updateType;
    LinkVisual::RenderMode renderMode;
};

class KinBodyVisual : public QObject
{
    Q_OBJECT

public:
    KinBodyVisual(Ogre::SceneManager *sceneManager,
                  Ogre::SceneNode *parentNode,
                  OpenRAVE::KinBodyPtr kinBody);
    virtual ~KinBodyVisual();

    void CreateProperties(rviz::Property *parent);

    inline OpenRAVE::KinBodyPtr GetKinBody() { return m_kinBody.lock(); }
    inline void SetKinBody(OpenRAVE::KinBodyPtr value) { m_kinBody = value; }

    inline Ogre::SceneManager* GetSceneManager() { return m_sceneManager; }
    inline void SetSceneManager(Ogre::SceneManager* value) { m_sceneManager = value; }

    inline Ogre::SceneNode* GetSceneNode() { return m_sceneNode; }
    inline void SetSceneNode(Ogre::SceneNode* value) { m_sceneNode = value; }

    inline Ogre::SceneNode* GetParentNode() { return m_parentNode; }
    inline void SetParentNode(Ogre::SceneNode* value) { m_parentNode = value; }

    void UpdateTransforms();

    void CreateParts();

    inline LinkVisual::RenderMode GetRenderMode() { return m_renderMode; }
    void SetRenderMode(LinkVisual::RenderMode mode);

public Q_SLOTS:
    void UpdateVisible();

protected:
    OpenRAVE::KinBodyWeakPtr m_kinBody;
    Ogre::SceneManager* m_sceneManager;
    Ogre::SceneNode* m_sceneNode;
    Ogre::SceneNode* m_parentNode;
    std::vector<LinkVisual*> m_links;

    rviz::Property *m_property_parent;
    rviz::BoolProperty *m_property_enabled;
    rviz::EnumProperty *m_property_visual;
    rviz::VectorProperty *m_property_position;
    rviz::QuaternionProperty *m_property_orientation;

    bool m_visible;
    LinkVisual::RenderMode m_renderMode;
};

}
#endif
