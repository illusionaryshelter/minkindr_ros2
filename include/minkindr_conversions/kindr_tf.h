#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <tf2/transform_datatypes.h>

#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace tf {

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void quaternionKindrToTF(
    const kindr::minimal::RotationQuaternionTemplate<Scalar> &kindr,
    tf2::Quaternion *tf_type) {
  CHECK_NOTNULL(tf_type);
  (*tf_type)[0] = kindr.toImplementation().x();
  (*tf_type)[1] = kindr.toImplementation().y();
  (*tf_type)[2] = kindr.toImplementation().z();
  (*tf_type)[3] = kindr.toImplementation().w();
}

template <typename Scalar>
void quaternionTFToKindr(
    const tf2::Quaternion &tf_type,
    kindr::minimal::RotationQuaternionTemplate<Scalar> *kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<Scalar> quat;
  quaternionTFToEigen(tf_type, quat);
  *kindr = kindr::minimal::RotationQuaternionTemplate<Scalar>(quat);
}
// Also the Eigen implementation version of this.
template <typename Scalar>
void quaternionKindrToTF(const Eigen::Quaternion<Scalar> &kindr,
                         tf2::Quaternion *tf_type) {
  CHECK_NOTNULL(tf_type);
  (*tf_type)[0] = kindr.x();
  (*tf_type)[1] = kindr.y();
  (*tf_type)[2] = kindr.z();
  (*tf_type)[3] = kindr.w();
}

template <typename Scalar>
void quaternionTFToKindr(const tf2::Quaternion &tf_type,
                         Eigen::Quaternion<Scalar> *kindr) {

  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond kindr_double(tf_type[3], tf_type[0], tf_type[1],
                                  tf_type[2]);
  *kindr = kindr_double.cast<Scalar>();
}

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void vectorKindrToTF(const Eigen::Matrix<Scalar, 3, 1> &kindr,
                     tf2::Vector3 *tf_type) {
  CHECK_NOTNULL(tf_type);
  (*tf_type)[0] = kindr(0);
  (*tf_type)[1] = kindr(1);
  (*tf_type)[2] = kindr(2);
}

template <typename Scalar>
void vectorTFToKindr(const tf2::Vector3 &tf_type,
                     Eigen::Matrix<Scalar, 3, 1> *kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<double, 3, 1> kindr_double;
  kindr_double(0) = tf_type[0];
  kindr_double(1) = tf_type[1];
  kindr_double(2) = tf_type[2];
  *kindr = kindr_double.cast<Scalar>();
}

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::Transform.
template <typename Scalar>
void transformKindrToTF(
    const kindr::minimal::QuatTransformationTemplate<Scalar> &kindr,
    tf2::Transform *tf_type) {
  CHECK_NOTNULL(tf_type);
  tf2::Vector3 origin;
  tf2::Quaternion rotation;
  vectorKindrToTF(kindr.getPosition(), &origin);
  quaternionKindrToTF(kindr.getRotation(), &rotation);
  tf_type->setOrigin(origin);
  tf_type->setRotation(rotation);
}

template <typename Scalar>
void transformTFToKindr(
    const tf2::Transform &tf_type,
    kindr::minimal::QuatTransformationTemplate<Scalar> *kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionTFToKindr(tf_type.getRotation(), &rotation);
  vectorTFToKindr(tf_type.getOrigin(), &position);

  // Enforce positive w.
  if (rotation.w() < 0) {
    rotation.coeffs() = -rotation.coeffs();
  }

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
template <typename Scalar>
void poseKindrToTF(
    const kindr::minimal::QuatTransformationTemplate<Scalar> &kindr,
    tf2::Transform *tf_type) {
  transformKindrToTF(kindr, tf_type);
}

template <typename Scalar>
void poseTFToKindr(const tf2::Transform &tf_type,
                   kindr::minimal::QuatTransformationTemplate<Scalar> *kindr) {
  transformTFToKindr(tf_type, kindr);
}

} // namespace tf
