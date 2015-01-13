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
#include "util/ros_conversions.h"

namespace or_interactivemarker {
namespace util {

OpenRAVE::geometry::RaveVector<double> ToRaveVector(Ogre::Vector3 const & vec)
{
    return OpenRAVE::geometry::RaveVector<double>(vec.x, vec.y, vec.z);
}

Ogre::Vector3 ToOgreVector(OpenRAVE::geometry::RaveVector<double> const &vec)
{
    return Ogre::Vector3(vec.x, vec.y, vec.z);
}

OpenRAVE::geometry::RaveVector<double> ToRaveQuaternion(Ogre::Quaternion const &quat)
{
    return OpenRAVE::geometry::RaveVector<double>(quat.x, quat.y, quat.z, quat.w);
}

Ogre::Quaternion ToOgreQuaternion(OpenRAVE::geometry::RaveVector<double> const &vec)
{
    return Ogre::Quaternion(vec.x, vec.y, vec.z, vec.w);
}

geometry_msgs::Pose ToGeomMsgPose(OpenRAVE::geometry::RaveTransform<double> const &transform)
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


OpenRAVE::geometry::RaveTransform<double> ToRaveTransform(geometry_msgs::Pose const &pose)
{
    OpenRAVE::geometry::RaveVector<double> const quat(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );
    OpenRAVE::geometry::RaveVector<double> const translation(
        pose.position.x,
        pose.position.y,
        pose.position.z
    );

    return OpenRAVE::geometry::RaveTransform<double>(quat, translation);
}

}
}
