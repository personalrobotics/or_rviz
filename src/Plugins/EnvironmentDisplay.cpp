/*
 * EnvironmentDisplay.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: mklingen
 */

#include "EnvironmentDisplay.h"
#include <rviz/frame_manager.h>
#include <rviz/properties/property_manager.h>


namespace or_rviz
{

    EnvironmentDisplay::EnvironmentDisplay() :
            m_frameProperty()
    {
        m_frame = "/map";
    }

    EnvironmentDisplay::~EnvironmentDisplay()
    {
        m_sceneManager->destroySceneNode(m_sceneNode);
        Clear();
    }

    void EnvironmentDisplay::UpdateObjects()
    {
        if(!GetEnvironment())
        {
            return;
        }

        std::vector<OpenRAVE::KinBodyPtr> bodies;
        GetEnvironment()->GetBodies(bodies);

        for(size_t i = 0; i < bodies.size(); i++)
        {
            if(!HasKinBody(bodies[i]->GetName()))
            {
                KinBodyVisual* visual = new KinBodyVisual(m_sceneManager, m_sceneNode, bodies.at(i));
                m_bodyVisuals[bodies[i]->GetName()] = visual;


                visual->SetCategory(
                        property_manager_->createCheckboxCategory(bodies.at(i)->GetName(),
                                                                 bodies.at(i)->GetName(),
                                                                 property_prefix_,
                                                                 boost::bind(&KinBodyVisual::IsVisible, visual),
                                                                 boost::bind(&KinBodyVisual::SetVisible, visual, _1),
                                                                 m_kinbodiesCategory, visual));

            }
            else
            {
                m_bodyVisuals[bodies[i]->GetName()]->UpdateTransforms();
            }
        }

        std::vector<std::string> removals;
        for(std::map<std::string, KinBodyVisual*>::iterator it = m_bodyVisuals.begin(); it != m_bodyVisuals.end(); it++)
        {
          bool containsBody = false;
          for(size_t j = 0; j < bodies.size(); j++)
          {
              if(bodies[j]->GetName() == it->first)
              {
                  containsBody = true;
                  break;
              }
          }

          if(!containsBody)
          {
              removals.push_back(it->first);
          }
        }

        for(size_t i = 0; i < removals.size(); i++)
        {
            RemoveKinBody(removals.at(i));
        }

    }

    void EnvironmentDisplay::RemoveKinBody(const std::string& name)
    {
        property_manager_->deleteProperty(m_bodyVisuals[name]->GetCategory().lock());
        delete m_bodyVisuals.at(name);
        m_bodyVisuals.erase(name);
    }

    void EnvironmentDisplay::Clear()
    {
        std::vector<std::string> removals;
        for(std::map<std::string, KinBodyVisual*>::iterator it = m_bodyVisuals.begin(); it != m_bodyVisuals.end(); it++)
         {
            removals.push_back(it->first);
         }

        for(size_t i = 0; i < removals.size(); i++)
        {
            RemoveKinBody(removals.at(i));
        }

        removals.clear();
    }

    void  EnvironmentDisplay::onInitialize()
    {
        m_sceneManager = this->scene_manager_;
        m_sceneNode = m_sceneManager->getRootSceneNode()->createChildSceneNode();
        RAVELOG_INFO("initialized\n");
    }

    void  EnvironmentDisplay::fixedFrameChanged()
    {
        rviz::FrameManager* frameManager = this->vis_manager_->getFrameManager();

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if(frameManager->getTransform(m_frame, ros::Time::now(), position, orientation))
        {
            m_sceneNode->setPosition(position);
            m_sceneNode->setOrientation(orientation);
        }
    }

    void  EnvironmentDisplay::reset()
    {
        Clear();
        UpdateObjects();
    }

    void  EnvironmentDisplay::createProperties()
    {
        m_frameProperty = property_manager_->createProperty<rviz::TFFrameProperty>("Frame", property_prefix_,
                boost::bind( &EnvironmentDisplay::GetFrame, this ),
                boost::bind( &EnvironmentDisplay::SetFrame, this, _1 ),
                parent_category_);

        m_kinbodiesCategory = property_manager_->createCategory("Kinbodies", property_prefix_, parent_category_, NULL);

    }

    void EnvironmentDisplay::onEnable()
    {
        m_sceneNode->setVisible(true, true);

        for(std::map<std::string, KinBodyVisual*>::iterator it = m_bodyVisuals.begin(); it != m_bodyVisuals.end(); it++)
         {
            it->second->SetVisible(true);
         }
    }

    void EnvironmentDisplay::onDisable()
    {
        m_sceneNode->setVisible(false, true);

        for(std::map<std::string, KinBodyVisual*>::iterator it = m_bodyVisuals.begin(); it != m_bodyVisuals.end(); it++)
         {
            it->second->SetVisible(false);
         }
    }

} /* namespace superviewer */


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( or_rviz, Environment, or_rviz::EnvironmentDisplay, rviz::Display )
