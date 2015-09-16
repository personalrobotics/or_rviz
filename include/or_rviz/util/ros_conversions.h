/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>

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
#ifndef ROS_CONVERSIONS_H_
#define ROS_CONVERSIONS_H_
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
// workaround for qt moc bug w.r.t. BOOST_JOIN macro
// see https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <openrave/openrave.h>
#endif

namespace or_rviz {
namespace util {

extern std::string const kDefaultWorldFrameId;

// OpenRAVE to ROS
template <class Scalar>
std_msgs::ColorRGBA toROSColor(OpenRAVE::RaveVector<Scalar> const &color);
template <class Scalar>
geometry_msgs::Vector3 toROSVector(OpenRAVE::RaveVector<Scalar> const &or_vector);
template <class Scalar>
geometry_msgs::Pose toROSPose(OpenRAVE::RaveTransform<Scalar> const &or_pose);
template <class Scalar>
geometry_msgs::Point toROSPoint(OpenRAVE::RaveVector<Scalar> const &or_point);
template <class Scalar>
geometry_msgs::Quaternion toROSQuaternion(OpenRAVE::RaveVector<Scalar> const &or_quat);

// ROS to OpenRAVE
template <class Scalar>
OpenRAVE::RaveVector<Scalar> toORPoint(geometry_msgs::Point const &point);
template <class Scalar>
OpenRAVE::RaveVector<Scalar> toORQuaternion(geometry_msgs::Quaternion const &quat);
template <class Scalar>
OpenRAVE::RaveTransform<Scalar> toORPose(geometry_msgs::Pose const &pose);

}
}

#endif
