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
#include "util/ogre_conversions.h"

using OpenRAVE::geometry::RaveVector;
using OpenRAVE::geometry::RaveTransform;

namespace or_rviz {
namespace util {

template <class Scalar>
RaveVector<Scalar> toORVector(Ogre::Vector3 const &vec)
{
    return RaveVector<Scalar>(vec.x, vec.y, vec.z);
}

template <class Scalar>
Ogre::Vector3 toOgreVector(RaveVector<Scalar> const &vec)
{
    return Ogre::Vector3(vec.x, vec.y, vec.z);
}

template <class Scalar>
RaveVector<Scalar> toORQuaternion(Ogre::Quaternion const &quat)
{
    RaveVector<Scalar> or_quat;
    or_quat[0] = quat.w;
    or_quat[1] = quat.x;
    or_quat[2] = quat.y;
    or_quat[3] = quat.z;
    return or_quat;
}

template <class Scalar>
Ogre::Quaternion toOgreQuaternion(RaveVector<Scalar> const &vec)
{
    Ogre::Quaternion ogre_quat;
    ogre_quat.w = vec[0];
    ogre_quat.x = vec[1];
    ogre_quat.y = vec[2];
    ogre_quat.z = vec[3];
    return ogre_quat;
}

// Explicit instantiations.
template RaveVector<float> toORVector<float>(Ogre::Vector3 const &vec);
template Ogre::Vector3 toOgreVector(RaveVector<float> const &vec);
template RaveVector<float> toORQuaternion(Ogre::Quaternion const &quat);
template Ogre::Quaternion toOgreQuaternion(RaveVector<float> const &vec);

template RaveVector<double> toORVector<double>(Ogre::Vector3 const &vec);
template Ogre::Vector3 toOgreVector(RaveVector<double> const &vec);
template RaveVector<double> toORQuaternion(Ogre::Quaternion const &quat);
template Ogre::Quaternion toOgreQuaternion(RaveVector<double> const &vec);

}
}
