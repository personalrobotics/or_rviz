#ifndef ROS_CONVERSIONS_H_
#define ROS_CONVERSIONS_H_

#include <OgrePose.h>
#include <geometry_msgs/Pose.h>
#include <openrave/geometry.h>

namespace or_interactivemarker {
namespace util {

OpenRAVE::geometry::RaveVector<double> ToRaveVector(Ogre::Vector3 const &vec);
Ogre::Vector3 ToOgreVector(OpenRAVE::geometry::RaveVector<double> const &vec);

OpenRAVE::geometry::RaveVector<double> ToRaveQuaternion(Ogre::Quaternion const &quat);
Ogre::Quaternion ToOgreQuaternion(OpenRAVE::geometry::RaveVector<double> const &vec);

geometry_msgs::Pose ToGeomMsgPose(OpenRAVE::geometry::RaveTransform<double> const &transform);
OpenRAVE::geometry::RaveTransform<double> ToRaveTransform(geometry_msgs::Pose const &pose);

}
}

#endif
