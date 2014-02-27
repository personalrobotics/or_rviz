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
//#include "../PythonInterface/PyCallbacks.h"


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

    bool  EnvironmentDisplay::RegisterMenuCallback(const std::string& objectName, const std::string& menuName, const std::string& pyObject)
    {
        m_pythonCallbacks[objectName][menuName] = pyObject;
        return true;
    }

    bool  EnvironmentDisplay::UnRegisterMenuCallback(const std::string& objectName, const std::string& menuName)
    {
        m_pythonCallbacks[objectName].erase(menuName);
        return true;
    }


    void EnvironmentDisplay::OnKinbodyMenuJointControlChanged(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
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

        control_mode::ControlMode type = (state == MenuHandler::UNCHECKED) ?  control_mode::JointControl : control_mode::NoControl;

        CreateControls(m_bodyVisuals[objectName], type, false);
        GetMenu(objectName).apply(*m_markerServer, objectName);
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

        control_mode::ControlMode type = (state == MenuHandler::UNCHECKED) ?  control_mode::PoseControl : control_mode::NoControl;

        CreateControls(m_bodyVisuals[objectName], type, false);
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

        m_bodyVisuals[objectName]->SetVisible(state == MenuHandler::UNCHECKED);
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
            BodyUpdateEvent updateEvent;
            updateEvent.bodyName = objectName;
            updateEvent.renderMode = state == MenuHandler::UNCHECKED ? LinkVisual::CollisionMesh : LinkVisual::VisualMesh;
            updateEvent.updateType = BodyUpdateRenderMode;
            m_updateBuffer.push_back(updateEvent);
        }

        GetMenu(objectName).apply(*m_markerServer, objectName);
    }

    void EnvironmentDisplay::CreateJointControl(visualization_msgs::InteractiveMarker& marker, OpenRAVE::KinBody::JointPtr& joint)
    {
        marker.scale = fmax(fmin((joint->GetHierarchyParentLink()->ComputeAABB().extents * 0.5).lengthsqr3(), 0.5), 0.2);
        if(joint->GetType() == OpenRAVE::KinBody::Joint::JointRevolute ||joint->GetType() == OpenRAVE::KinBody::Joint::JointPrismatic )
        {
            CreateJointDOFControl(marker, joint->GetAxis(0), joint->GetDOFIndex(), 0, joint->GetType());
        }
        else
        {
            int numDofs = joint->GetInfo()._vaxes.size();
            for(int i = 0; i < numDofs; i++)
            {
                bool revolute = joint->IsRevolute(i);
                bool prismatic = joint->IsPrismatic(i);
                OpenRAVE::Vector axis = joint->GetAxis(i);

                OpenRAVE::KinBody::Joint::JointType type;

                if(revolute)
                {
                    type = OpenRAVE::KinBody::Joint::JointRevolute;
                }
                else if(prismatic)
                {
                    type = OpenRAVE::KinBody::Joint::JointPrismatic;
                }
                else
                {
                    continue;
                }

                CreateJointDOFControl(marker, axis, joint->GetDOFIndex(), i, type);
            }
        }


    }

    void EnvironmentDisplay::CreateJointDOFControl(visualization_msgs::InteractiveMarker& int_marker, const OpenRAVE::Vector& axis,  int jointID, int dofID,  OpenRAVE::KinBody::Joint::JointType type)
    {
        std::stringstream ss;
        ss << jointID << "," << dofID;

        RAVELOG_INFO("ID: %s" , ss.str().c_str());

        OpenRAVE::Vector axisAngle;
        axisAngle = axis;
        axisAngle.w = 0;


        Ogre::Quaternion ogreQuat;
        ogreQuat.w = 1;
        ogreQuat.x = 0;
        ogreQuat.y = 1;
        ogreQuat.z = 0;

        if(type == OpenRAVE::KinBody::Joint::JointRevolute)
        {
            InteractiveMarkerControl control;
            control.orientation.w = ogreQuat.w;
            control.orientation.x = ogreQuat.x;
            control.orientation.y = ogreQuat.y;
            control.orientation.z = ogreQuat.z;
            control.name = ss.str();

            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
        }
        else if(type == OpenRAVE::KinBody::Joint::JointPrismatic)
        {
            InteractiveMarkerControl control;
            control.orientation.w = ogreQuat.w;
            control.orientation.x = ogreQuat.x;
            control.orientation.y = ogreQuat.y;
            control.orientation.z = ogreQuat.z;
            control.name = ss.str();



            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
        }
        else
        {
            RAVELOG_WARN("Unrecognized joint type %d\n", (int)type);
        }


    }

    void EnvironmentDisplay::CreatePoseControl(visualization_msgs::InteractiveMarker& int_marker)
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

    void EnvironmentDisplay::UpdateJointControlPoses(KinBodyVisual* visual)
    {
        for(size_t i = 0; i < visual->GetKinBody()->GetJoints().size(); i++)
        {
            InteractiveMarker joint_marker;
            OpenRAVE::KinBody::JointPtr joint = visual->GetKinBody()->GetJoints().at(i);
            if(!m_markerServer->get(visual->GetKinBody()->GetName() + " " + joint->GetName(), joint_marker))
            {
                continue;
            }

            joint_marker.header.frame_id = m_frame;
            OpenRAVE::Transform transform = ComputeFacingMatrix(joint->GetAxis(0));
            Ogre::Quaternion ogreQuat = converters::ToOgreQuaternion(transform.rot);

            joint_marker.pose.orientation.x = ogreQuat.x;
            joint_marker.pose.orientation.y = ogreQuat.y;
            joint_marker.pose.orientation.z = ogreQuat.z;
            joint_marker.pose.orientation.w = ogreQuat.w;
            joint_marker.pose.position.x = joint->GetAnchor().x;
            joint_marker.pose.position.y = joint->GetAnchor().y;
            joint_marker.pose.position.z = joint->GetAnchor().z;

            joint_marker.name = visual->GetKinBody()->GetName() + " " + joint->GetName();

            m_markerServer->setPose(joint_marker.name, joint_marker.pose);


        }
    }

    void EnvironmentDisplay::CreateTransformController(visualization_msgs::InteractiveMarker& marker, const std::string& tfFrom, const std::string& tfTo, const std::string& fixedFrame)
    {
        tf::StampedTransform transform;


        bool success = false;


        while(!success)
        try
        {
            m_tfListener.lookupTransform("/calibrator", ros::Time(0), tfTo, ros::Time(0), "/calibrator", transform);
            success = true;
        }

        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        marker.header.frame_id = fixedFrame;
        marker.pose.position.x = transform.getOrigin().x();
        marker.pose.position.y = transform.getOrigin().y();
        marker.pose.position.z = transform.getOrigin().z();
        marker.pose.orientation.x = transform.getRotation().x();
        marker.pose.orientation.y = transform.getRotation().y();
        marker.pose.orientation.z = transform.getRotation().z();
        marker.pose.orientation.w = transform.getRotation().w();

        marker.name = "calibrator";
        marker.scale = 0.5f;

        CreatePoseControl(marker);
        m_markerServer->insert(marker);
        m_markerServer->setCallback(marker.name, boost::bind(&EnvironmentDisplay::OnCalibratorMoved, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
        ROS_INFO("Created calibrator");
    }

    void EnvironmentDisplay::CreateControls(KinBodyVisual* visual, control_mode::ControlMode mode, bool immediate)
    {
        if(!visual || !m_markerServer)
        {
            return;
        }

        if(!immediate)
        {
            ControlHandle controlHandle;
            controlHandle.mode = mode;
            controlHandle.name = visual->GetKinBody()->GetName();
            m_controlBuffer.push_back(controlHandle);
            return;
        }

        MenuHandler& menu = GetMenu(visual->GetKinBody()->GetName());


        menu = MenuHandler();

        menu.setCheckState(menu.insert("Visible",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuVisibleChanged, this, _1)),
                visual->IsVisible() ?  MenuHandler::CHECKED : MenuHandler::UNCHECKED);


        menu.setCheckState(menu.insert("Collision Mesh",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuCollisionChanged, this, _1)),
                visual->GetRenderMode() == LinkVisual::CollisionMesh ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);


        menu.setCheckState(menu.insert("Move",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuMoveChanged, this, _1)),
                mode == control_mode::PoseControl ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);


        menu.setCheckState(menu.insert("Joints",
                boost::bind(&EnvironmentDisplay::OnKinbodyMenuJointControlChanged, this, _1)),
                mode == control_mode::JointControl ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);


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

        if(mode == control_mode::PoseControl)
        {
            CreatePoseControl(int_marker);
        }

        if(mode == control_mode::JointControl)
        {

            for(size_t i = 0; i < visual->GetKinBody()->GetJoints().size(); i++)
            {
                OpenRAVE::KinBody::JointPtr joint = visual->GetKinBody()->GetJoints().at(i);
                InteractiveMarker joint_marker;
                joint_marker.header.frame_id = m_frame;

                OpenRAVE::Transform transform = ComputeFacingMatrix(joint->GetAxis(0));
                transform.trans = joint->GetAnchor();
                Ogre::Quaternion ogreQuat = converters::ToOgreQuaternion(transform.rot);
                joint_marker.pose.orientation.x = ogreQuat.x;
                joint_marker.pose.orientation.y = ogreQuat.y;
                joint_marker.pose.orientation.z = ogreQuat.z;
                joint_marker.pose.orientation.w = ogreQuat.w;
                joint_marker.pose.position.x = joint->GetAnchor().x;
                joint_marker.pose.position.y = joint->GetAnchor().y;
                joint_marker.pose.position.z = joint->GetAnchor().z;

                joint_marker.name = visual->GetKinBody()->GetName() + " " + joint->GetName();


                CreateJointControl(joint_marker, joint);
                m_markerServer->insert(joint_marker);
                m_markerServer->setCallback(joint_marker.name, boost::bind(&EnvironmentDisplay::OnJointMoved, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
            }
        }
        else
        {
            for(size_t i = 0; i < visual->GetKinBody()->GetJoints().size(); i++)
            {
                OpenRAVE::KinBody::JointPtr joint = visual->GetKinBody()->GetJoints().at(i);
                m_markerServer->erase(visual->GetKinBody()->GetName() + " " + joint->GetName());
            }
        }

        m_markerServer->insert(int_marker);
        m_markerServer->setCallback(visual->GetKinBody()->GetName(), boost::bind(&EnvironmentDisplay::OnKinbodyMoved, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
        menu.apply(*m_markerServer, int_marker.name);

    }

    void EnvironmentDisplay::OnCalibratorMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        ROS_INFO("Calibrator moved");
        geometry_msgs::Pose pose = feedback->pose;
        tf::StampedTransform transform;
        transform.frame_id_ = feedback->header.frame_id;
        transform.stamp_ = ros::Time::now();
        transform.child_frame_id_ = "/calibrator";
        transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        m_tfBroadcaster.sendTransform(transform);
    }

    void EnvironmentDisplay::OnJointMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {

        try
        {
            std::string markerName = feedback->marker_name;
            std::vector<std::string> tokens;
            std::istringstream iss(markerName);
            std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                    std::back_inserter<std::vector<std::string> >(tokens));

            std::string kinbodyName = tokens[0];
            std::string jointName = tokens[1];

            std::string dofName = feedback->control_name;

            int index = GetEnvironment()->GetKinBody(kinbodyName)->GetJoint(jointName)->GetDOFIndex();
            double currentValue = GetEnvironment()->GetKinBody(kinbodyName)->GetJoint(jointName)->GetValue(0);


            OpenRAVE::KinBody::JointPtr joint = GetEnvironment()->GetKinBody(kinbodyName)->GetJoint(jointName);

            InteractiveMarker joint_marker;
            OpenRAVE::Transform transform = ComputeFacingMatrix(joint->GetAxis(0));
            Ogre::Quaternion ogreQuat = converters::ToOgreQuaternion(transform.rot);

            joint_marker.pose.orientation.x = ogreQuat.x;
            joint_marker.pose.orientation.y = ogreQuat.y;
            joint_marker.pose.orientation.z = ogreQuat.z;
            joint_marker.pose.orientation.w = ogreQuat.w;
            joint_marker.pose.position.x = joint->GetAnchor().x;
            joint_marker.pose.position.y = joint->GetAnchor().y;
            joint_marker.pose.position.z = joint->GetAnchor().z;


            OpenRAVE::Transform prevPose = converters::ToRaveTransform(joint_marker.pose);
            OpenRAVE::Transform newPose = converters::ToRaveTransform(feedback->pose);
            OpenRAVE::Transform prevToNew = newPose.inverse() * prevPose;


            float change = 0;
            if(joint->GetType() == OpenRAVE::KinBody::JointRevolute)
            {
                OpenRAVE::Vector axisAngle = OpenRAVE::geometry::axisAngleFromQuat(prevToNew.rot);

                float diff = axisAngle.lengthsqr3();
                axisAngle /= (axisAngle.lengthsqr3() + 0.001f);
                float sign = axisAngle.dot(joint->GetAxis(0));
                float alpha = -0.1;

                change = diff * sign * alpha;
            }
            else
            {
                change = prevToNew.trans.lengthsqr3();
            }

            std::vector<double> values;
            values.push_back(currentValue + change);
            std::vector<int> indecies;
            indecies.push_back(index);

            GetEnvironment()->GetKinBody(kinbodyName)->SetDOFValues(values, true, indecies);

            UpdateJointControlPoses(m_bodyVisuals[kinbodyName]);
        }
        catch(OpenRAVE::openrave_exception& e)
        {
            RAVELOG_ERROR("%s\n", e.what());
        }

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


    void EnvironmentDisplay::HandleControlBufferEvents()
    {
        for(size_t i =0 ; i < m_controlBuffer.size(); i++)
        {
            ControlHandle& controlHandle = m_controlBuffer.at(i);

            if(HasKinBody(controlHandle.name))
            {
                CreateControls(m_bodyVisuals[controlHandle.name], controlHandle.mode, true);
            }

        }
        m_controlBuffer.clear();
    }

    void EnvironmentDisplay::HandleBodyUpdateEvents()
    {
        for(size_t i = 0; i < m_updateBuffer.size(); i++)
        {
            BodyUpdateEvent& updateEvent= m_updateBuffer.at(i);

            switch(updateEvent.updateType)
            {
                case BodyUpdateRenderMode:
                    if(HasKinBody(updateEvent.bodyName))
                    {
                        m_bodyVisuals[updateEvent.bodyName]->SetRenderMode(updateEvent.renderMode);
                    }
                    break;
            }

        }

        m_updateBuffer.clear();
    }


    void EnvironmentDisplay::RemoveDeadBodies(std::vector<OpenRAVE::KinBodyPtr>& bodies)
    {
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
                CreateControls(visual, control_mode::NoControl, false);

            }
            else
            {
                /*
                if(m_bodyVisuals[bodies[i]->GetName()]->IsVisible() != bodies[i]->IsVisible())
                {
                    m_bodyVisuals[bodies[i]->GetName()]->SetVisible(bodies[i]->IsVisible());
                }
                */

                m_bodyVisuals[bodies[i]->GetName()]->UpdateTransforms();
                m_markerServer->setPose(bodies[i]->GetName(), converters::ToGeomMsgPose(bodies[i]->GetTransform()));


                UpdateJointControlPoses(m_bodyVisuals[bodies[i]->GetName()]);
            }
        }

        RemoveDeadBodies(bodies);
        HandleControlBufferEvents();
        HandleBodyUpdateEvents();


        m_markerServer->applyChanges();

    }

    void EnvironmentDisplay::RemoveKinBody(const std::string& name)
    {
        if(HasKinBody(name))
        {
            property_manager_->deleteProperty(m_bodyVisuals[name]->GetCategory().lock());
            delete m_bodyVisuals.at(name);
            m_bodyVisuals.erase(name);
            m_markerServer->erase(name);
        }
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
        /*
        InteractiveMarker marker;
        CreateTransformController(marker, "/head/wam2", "/head_kinect_link", "/herb_base");
        */

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
