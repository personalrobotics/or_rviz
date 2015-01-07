#include "util/or_conversions.h"

using OpenRAVE::dReal;
using OpenRAVE::RaveVector;
using OpenRAVE::RaveTransform;
using std_msgs::ColorRGBA;
using geometry_msgs::Vector3;
using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;

namespace or_interactivemarker {
namespace util {

/*
 * OpenRAVE to ROS
 */
template <class Scalar>
ColorRGBA toROSColor(RaveVector<Scalar> const &or_color)
{
    ColorRGBA color;
    color.r = or_color.x;
    color.g = or_color.y;
    color.b = or_color.z;
    color.a = or_color.w;
    return color;
}

template <class Scalar>
Vector3 toROSVector(RaveVector<Scalar> const &or_vector)
{
    Vector3 v;
    v.x = or_vector.x;
    v.y = or_vector.y;
    v.z = or_vector.z;
    return v;
}

template <class Scalar>
Pose toROSPose(RaveTransform<Scalar> const &or_pose)
{
    Pose pose;
    pose.position = toROSPoint<>(or_pose.trans);
    pose.orientation = toROSQuaternion<>(or_pose.rot);
    return pose;
}

template <class Scalar>
Point toROSPoint(RaveVector<Scalar> const &or_point)
{
    Point point;
    point.x = or_point.x;
    point.y = or_point.y;
    point.z = or_point.z;
    return point;
}

template <class Scalar>
Quaternion toROSQuaternion(RaveVector<Scalar> const &or_quat)
{
    Quaternion quaternion;
    quaternion.w = or_quat[0];
    quaternion.x = or_quat[1];
    quaternion.y = or_quat[2];
    quaternion.z = or_quat[3];
    return quaternion;
}

/*
 * ROS to OpenRAVE
 */
template <class Scalar>
RaveVector<Scalar> toORPoint(Point const &point)
{
    return RaveVector<Scalar>(point.x, point.y, point.z);
}

template <class Scalar>
RaveVector<Scalar> toORQuaternion(Quaternion const &quat)
{
    RaveVector<Scalar> or_quat;
    or_quat[0] = quat.w;
    or_quat[1] = quat.x;
    or_quat[2] = quat.y;
    or_quat[3] = quat.z;
    return or_quat;
}

template <class Scalar>
RaveTransform<Scalar> toORPose(Pose const &pose)
{
    RaveTransform<Scalar> or_transform;
    or_transform.trans = toORPoint<Scalar>(pose.position);
    or_transform.rot = toORQuaternion<Scalar>(pose.orientation);
    return or_transform;
}

// Explicit instantiations.
template ColorRGBA toROSColor<float>(RaveVector<float> const &color);
template Vector3 toROSVector<float>(RaveVector<float> const &or_vector);
template Pose toROSPose<float>(RaveTransform<float> const &or_pose);
template Point toROSPoint<float>(RaveVector<float> const &or_point);
template Quaternion toROSQuaternion<float>(RaveVector<float> const &or_point);
template RaveVector<float> toORPoint(Point const &point);
template RaveVector<float> toORQuaternion(Quaternion const &quat);
template RaveTransform<float> toORPose(Pose const &pose);

template ColorRGBA toROSColor<double>(RaveVector<double> const &color);
template Vector3 toROSVector<double>(RaveVector<double> const &or_vector);
template Pose toROSPose<double>(RaveTransform<double> const &or_pose);
template Point toROSPoint<double>(RaveVector<double> const &or_point);
template Quaternion toROSQuaternion<double>(RaveVector<double> const &or_point);
template RaveVector<double> toORPoint(Point const &point);
template RaveVector<double> toORQuaternion(Quaternion const &quat);
template RaveTransform<double> toORPose(Pose const &pose);

}
}
