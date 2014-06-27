/*
 * KinBodyVisual.h
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#ifndef KINBODYVISUAL_H_
#define KINBODYVISUAL_H_

#include <openrave/openrave.h>
#include <openrave/kinbody.h>
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
            KinBodyVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, OpenRAVE::KinBodyPtr kinBody);
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

} /* namespace superviewer */
#endif /* KINBODYVISUAL_H_ */
