/*
 * EnvironmentDisplay.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: mklingen
 */

#include "EnvironmentDisplay.h"
#include <rviz/frame_manager.h>
#include <rviz/properties/property_manager.h>
#include "../Converters.h"

using namespace interactive_markers;
using namespace visualization_msgs;


namespace or_rviz
{

    EnvironmentDisplay::EnvironmentDisplay() :
            m_frameProperty()
    {
        m_frame = "/map";
        m_markerServer = new interactive_markers::InteractiveMarkerServer("openrave_markers", "", true);

    }

    EnvironmentDisplay::~EnvironmentDisplay()
    {
        Clear();
        m_sceneManager->destroySceneNode(m_sceneNode);
        delete m_markerServer;
    }

    void EnvironmentDisplay::OnKinbodyMenuMoveChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {

        std::string objectName = feedback->marker_name;
        MenuHandler::EntryHandle handle = feedback->menu_entry_id;
        MenuHandler::CheckState state;
        GetMenu(objectName).getCheckState( handle, state );

        if(state == MenuHandler::CHECKED)
        {
            GetMenu(objectName).setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
            GetMenu(objectName).setCheckState(handle, MenuHandler::CHECKED);
        }

        CreateControls(m_bodyVisuals[objectName], state == MenuHandler::UNCHECKED, false);
        GetMenu(objectName).apply(*m_markerServer, objectName);

    }

    void EnvironmentDisplay::OnKinbodyMenuVisibleChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {

        std::string objectName = feedback->marker_name;
        OpenRAVE::KinBodyPtr body = GetEnvironment()->GetKinBody(objectName);

        if(!body.get())
        {
            return;
        }

        MenuHandler::EntryHandle handle = feedback->menu_entry_id;
        MenuHandler::CheckState state;
        GetMenu(objectName).getCheckState( handle, state );

        if(state == MenuHandler::CHECKED)
        {
            GetMenu(objectName).setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
            GetMenu(objectName).setCheckState(handle, MenuHandler::CHECKED);
        }

        GetEnvironment()->GetKinBody(objectName)->SetVisible(state == MenuHandler::UNCHECKED);
        GetMenu(objectName).apply(*m_markerServer, objectName);

    }

    void EnvironmentDisplay::OnKinbodyMenuDelete(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {

        std::string objectName = feedback->marker_name;
        OpenRAVE::KinBodyPtr body = GetEnvironment()->GetKinBody(objectName);

        if(body.get())
        {
            GetEnvironment()->Remove(body);
            RAVELOG_INFO("Deleting %s\n", objectName.c_str());
        }

    }

    void  EnvironmentDisplay::OnKinbodyMenuCollisionChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        std::string objectName = feedback->marker_name;
        MenuHandler::EntryHandle handle = feedback->menu_entry_id;
        MenuHandler::CheckState state;
        GetMenu(objectName).getCheckState( handle, state );

        if(state == MenuHandler::CHECKED)
        {
            GetMenu(objectName).setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
            GetMenu(objectName).setCheckState(handle, MenuHandler::CHECKED);
        }

        if(HasKinBody(objectName))
        {
            m_bodyVisuals[objectName]->SetRenderMode(state == MenuHandler::CHECKED ? LinkVisual::CollisionMesh : LinkVisual::VisualMesh);
        }

        GetMenu(objectName).apply(*m_markerServer, objectName);
    }

    void EnvironmentDisplay::CreateControls(KinBodyVisual* visual, bool poseControl, bool immediate)
    {
        if(!visual || !m_markerServer)
        {
            return;
        }

        if(!immediate)
        {
            ControlHandle controlHandle;
            controlHandle.createPose = poseControl;
            controlHandle.name = visual->GetKinBody()->GetName();
            m_controlBuffer.push_back(controlHandle);
            return;
        }

        MenuHandler& menu = GetMenu(visual->GetKinBody()->GetName());


        menu = MenuHandler();

        menu.setCheckState(menu.insert("Visible",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuVisibleChanged, this, _1)),
                MenuHandler::CHECKED);

        /*
        menu.setCheckState(menu.insert("Collision Mesh",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuCollisionChanged, this, _1)),
                MenuHandler::UNCHECKED);
                */

        menu.setCheckState(menu.insert("Move",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuMoveChanged, this, _1)),
                (!poseControl) ? MenuHandler::UNCHECKED : MenuHandler::CHECKED);
        menu.insert("Delete",
                        boost::bind(&EnvironmentDisplay::OnKinbodyMenuDelete, this, _1));


        InteractiveMarker int_marker;
        int_marker.header.frame_id = m_frame;
        int_marker.name = visual->GetKinBody()->GetName();



        InteractiveMarkerControl menuControl;
        menuControl.interaction_mode = InteractiveMarkerControl::MENU;
        menuControl.name = visual->GetKinBody()->GetName();
        menuControl.description = visual->GetKinBody()->GetName();
        int_marker.controls.push_back(menuControl);



        OpenRAVE::AABB aabb = visual->GetKinBody()->ComputeAABB();

        int_marker.scale = sqrt((aabb.extents * 1.5).lengthsqr3());

        OpenRAVE::Transform transform = visual->GetKinBody()->GetTransform();
        transform.trans = aabb.pos;
        int_marker.pose = converters::ToGeomMsgPose(transform);

        if(poseControl)
        {
            InteractiveMarkerControl control;

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);

            control.name = "move_x";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);

            control.name = "move_z";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);

            control.name = "move_y";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
        }

        m_markerServer->insert(int_marker);
        m_markerServer->setCallback(visual->GetKinBody()->GetName(), boost::bind(&EnvironmentDisplay::OnKinbodyMoved, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
        menu.apply(*m_markerServer, int_marker.name);

    }

    void  EnvironmentDisplay::OnKinbodyMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        std::string objectName = feedback->marker_name;

        if(GetEnvironment()->GetKinBody(objectName).get())
        {
            GetEnvironment()->GetKinBody(objectName)->SetTransform(converters::ToRaveTransform(feedback->pose));
        }
    }

    void EnvironmentDisplay::CreateRvizPropertyMenu(KinBodyVisual* visual)
    {
        visual->SetCategory(
                property_manager_->createCheckboxCategory(visual->GetKinBody()->GetName(),
                                                          visual->GetKinBody()->GetName(),
                                                         property_prefix_,
                                                         boost::bind(&KinBodyVisual::IsVisible, visual),
                                                         boost::bind(&KinBodyVisual::SetVisible, visual, _1),
                                                         m_kinbodiesCategory, visual));
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
                visual->SetVisible(true);

                m_bodyVisuals[bodies[i]->GetName()] = visual;


                CreateRvizPropertyMenu(visual);
                CreateControls(visual, false, false);

            }
            else
            {
                if(m_bodyVisuals[bodies[i]->GetName()]->IsVisible() != bodies[i]->IsVisible())
                {
                    m_bodyVisuals[bodies[i]->GetName()]->SetVisible(bodies[i]->IsVisible());
                }

                m_bodyVisuals[bodies[i]->GetName()]->UpdateTransforms();
                m_markerServer->setPose(bodies[i]->GetName(), converters::ToGeomMsgPose(bodies[i]->GetTransform()));
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


        for(size_t i =0 ; i < m_controlBuffer.size(); i++)
        {
            ControlHandle& controlHandle = m_controlBuffer.at(i);

            if(HasKinBody(controlHandle.name))
            {
                CreateControls(m_bodyVisuals[controlHandle.name], controlHandle.createPose, true);
            }

        }

        m_controlBuffer.clear();

        m_markerServer->applyChanges();

    }

    void EnvironmentDisplay::RemoveKinBody(const std::string& name)
    {
        property_manager_->deleteProperty(m_bodyVisuals[name]->GetCategory().lock());
        delete m_bodyVisuals.at(name);
        m_bodyVisuals.erase(name);
        m_markerServer->erase(name);
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
