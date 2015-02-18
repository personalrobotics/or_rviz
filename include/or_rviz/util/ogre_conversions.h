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
#ifndef OGRE_CONVERSIONS_H_
#define OGRE_CONVERSIONS_H_

#include <OgrePose.h>
#include <geometry_msgs/Pose.h>
#include <openrave/geometry.h>

namespace or_rviz {
namespace util {

template <class Scalar>
OpenRAVE::geometry::RaveVector<Scalar> toORVector(Ogre::Vector3 const &vec);
template <class Scalar>
Ogre::Vector3 toOgreVector(OpenRAVE::geometry::RaveVector<Scalar> const &vec);

template <class Scalar>
OpenRAVE::geometry::RaveVector<Scalar> toORQuaternion(Ogre::Quaternion const &quat);
template <class Scalar>
Ogre::Quaternion toOgreQuaternion(OpenRAVE::geometry::RaveVector<Scalar> const &vec);

}
}

#endif
