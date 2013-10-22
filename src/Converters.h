/*
 * Converters.h
 *
 *  Created on: Sep 20, 2013
 *      Author: mklingen
 */

#ifndef CONVERTERS_H_
#define CONVERTERS_H_

#include <openrave/geometry.h>

#include <OgrePose.h>
#include <geometry_msgs/Pose.h>

namespace converters
{
    inline OpenRAVE::geometry::RaveVector<double> ToRaveVector(const Ogre::Vector3& vec) { return  OpenRAVE::geometry::RaveVector<double>(vec.x, vec.y, vec.z); }
    inline Ogre::Vector3 ToOgreVector(const OpenRAVE::geometry::RaveVector<double>& vec) { return Ogre::Vector3(vec.x, vec.y, vec.z); }
    inline OpenRAVE::geometry::RaveVector<double> ToRaveQuaternion(const Ogre::Quaternion& quat) { return OpenRAVE::geometry::RaveVector<double>(quat.x, quat.y, quat.z, quat.w); }
    inline Ogre::Quaternion ToOgreQuaternion(const OpenRAVE::geometry::RaveVector<double>& vec) { return Ogre::Quaternion(vec.x, vec.y, vec.z, vec.w); }

    inline geometry_msgs::Pose ToGeomMsgPose(const OpenRAVE::geometry::RaveTransform<double>& transform)
    {
        geometry_msgs::Pose toReturn;
        toReturn.position.x = transform.trans.x;
        toReturn.position.y = transform.trans.y;
        toReturn.position.z = transform.trans.z;
        toReturn.orientation.w = transform.rot.x;
        toReturn.orientation.x = transform.rot.y;
        toReturn.orientation.y = transform.rot.z;
        toReturn.orientation.z = transform.rot.w;
        return toReturn;
    }


    inline OpenRAVE::geometry::RaveTransform<double> ToRaveTransform(const geometry_msgs::Pose& pose)
    {
        OpenRAVE::geometry::RaveVector<double> quat(pose.orientation.w, pose.orientation.x,pose.orientation.y,pose.orientation.z);
        OpenRAVE::geometry::RaveVector<double> translation(pose.position.x, pose.position.y, pose.position.z);
        return  OpenRAVE::geometry::RaveTransform<double>(quat, translation);
    }

}


#endif /* CONVERTERS_H_ */
