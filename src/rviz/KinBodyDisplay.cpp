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
