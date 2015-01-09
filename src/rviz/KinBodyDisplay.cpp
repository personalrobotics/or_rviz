/*
 * KinBodyDisplay.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <pluginlib/class_list_macros.h>
#include "rviz/Converters.h"
#include "rviz/KinBodyDisplay.h"


namespace or_rviz
{

KinBodyDisplay::KinBodyDisplay()
    : m_visual(NULL)
{
}

KinBodyDisplay::KinBodyDisplay(OpenRAVE::KinBodyPtr kinBod, Ogre::SceneManager* sceneManager)
    : m_kinbody(kinBod)
    , m_visual(NULL)
{
    CreateVisual(m_kinbody, sceneManager);
}

void KinbodyDisplay::CreateProperties(rviz::Property *parent)
{
    BOOST_ASSERT(m_kinbody);
    RAVELOG_INFO("CreateProperties: %s\n", m_kinbody->GetName().c_str());
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

}

PLUGINLIB_DECLARE_CLASS( or_rviz, KinBody, or_rviz::KinBodyDisplay, rviz::Display );
