#ifndef ORCONVERSIONS_H_
#define ORCONVERSIONS_H_
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <openrave/openrave.h>

namespace or_interactivemarker {
namespace util {

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
