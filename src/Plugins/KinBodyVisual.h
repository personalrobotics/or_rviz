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

namespace Ogre
{
    class SceneManager;
    class SceneNode;
}


namespace superviewer
{

    class LinkVisual;
    class KinBodyVisual
    {
        public:
            KinBodyVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, OpenRAVE::KinBodyPtr kinBody);
            virtual ~KinBodyVisual();

            inline OpenRAVE::KinBodyPtr GetKinBody() { return m_kinBody; }
            inline void SetKinBody(OpenRAVE::KinBodyPtr value) { m_kinBody = value; }

            inline Ogre::SceneManager* GetSceneManager() { return m_sceneManager; }
            inline void SetSceneManager(Ogre::SceneManager* value) { m_sceneManager = value; }

            inline Ogre::SceneNode* GetSceneNode() { return m_sceneNode; }
            inline void SetSceneNode(Ogre::SceneNode* value) { m_sceneNode = value; }

            inline Ogre::SceneNode* GetParentNode() { return m_parentNode; }
            inline void SetParentNode(Ogre::SceneNode* value) { m_parentNode = value; }

            void CreateParts();

        protected:
            OpenRAVE::KinBodyPtr m_kinBody;
            Ogre::SceneManager* m_sceneManager;
            Ogre::SceneNode* m_sceneNode;
            Ogre::SceneNode* m_parentNode;
            std::vector<LinkVisual*> m_links;

    };

} /* namespace superviewer */
#endif /* KINBODYVISUAL_H_ */
