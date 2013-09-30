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

namespace converters
{
    inline OpenRAVE::geometry::RaveVector<double> ToRaveVector(const Ogre::Vector3& vec) { return  OpenRAVE::geometry::RaveVector<double>(vec.x, vec.y, vec.z); }
    inline Ogre::Vector3 ToOgreVector(const OpenRAVE::geometry::RaveVector<double>& vec) { return Ogre::Vector3(vec.x, vec.y, vec.z); }
    inline OpenRAVE::geometry::RaveVector<double> ToRaveQuaternion(const Ogre::Quaternion& quat) { return OpenRAVE::geometry::RaveVector<double>(quat.x, quat.y, quat.z, quat.w); }
    inline Ogre::Quaternion ToOgreQuaternion(const OpenRAVE::geometry::RaveVector<double>& vec) { return Ogre::Quaternion(vec.x, vec.y, vec.z, vec.w); }


}


#endif /* CONVERTERS_H_ */
