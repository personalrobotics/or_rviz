/*
 * KinBodyDisplay.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#include "../Converters.h"
#include "KinBodyDisplay.h"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <pluginlib/class_list_macros.h>


namespace or_rviz
{

KinBodyDisplay::KinBodyDisplay()
{
    m_visual = NULL;
}

KinBodyDisplay::KinBodyDisplay(OpenRAVE::KinBodyPtr kinBod, Ogre::SceneManager* sceneManager)
{
    CreateVisual(kinBod, sceneManager);
}

void KinBodyDisplay::CreateVisual(OpenRAVE::KinBodyPtr kinBody, Ogre::SceneManager* sceneManager)
{
    m_visual = new KinBodyVisual(sceneManager, sceneManager->getRootSceneNode(), kinBody);
}

KinBodyDisplay::~KinBodyDisplay()
{
    // Gets deleted by RVIZ
    delete m_visual;
}

void  KinBodyDisplay::onInitialize()
{
    //TODO: Implement
}

void  KinBodyDisplay::fixedFrameChanged()
{
    //TODO: Implement
}

void  KinBodyDisplay::reset()
{
    //TODO: Implement
}

void  KinBodyDisplay::createProperties()
{
    //TODO: Implement
}

void KinBodyDisplay::onEnable()
{
    if(m_visual && m_visual->GetSceneNode()) {
        m_visual->GetSceneNode()->setVisible(true, true);
    }
}

void KinBodyDisplay::onDisable()
{
    if(m_visual && m_visual->GetSceneNode()) {
        m_visual->GetSceneNode()->setVisible(false, true);
    }
}

}

PLUGINLIB_DECLARE_CLASS( or_rviz, KinBody, or_rviz::KinBodyDisplay, rviz::Display );
