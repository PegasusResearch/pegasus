/**
 * Authors:
 * 		André Potes (andre.potes@gmail.com)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: frames.hpp 
 * Brief: Defines all functions related to conversions between ENU do NED frames and vice-versa
 * 
 * NOTE: Most of this code is adapted from mavros
 * https://github.com/mavlink/mavros/blob/master/mavros/src/lib/ftf_frame_conversions.cpp
 * which had as authors Nuno Marques (n.marques21@hotmail.com) and Eddy Scott (scott.edward@aurora.aero)
 */
#pragma once

#include <Eigen/Dense>
#include "rotations.hpp"

namespace Pegasus {

namespace Frames {

/**
 * @brief Static quaternion to convert a rotation expressed in ENU to a rotation expressed in NED (Z->Y->X convention) on
 * 			the inertial frame
 * Rotate PI/2 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis
 * 
 * NOTE: this quaternion is as valid as the quaternion representing the rotation from NED to ENU (quaternion ambiguity) on
 * 			the inertial frame
 */
template <typename T>
static const Eigen::Quaternion<T> ENU_NED_INERTIAL_Q = Pegasus::Rotations::euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, M_PI_2));

/**
 * @brief Static quaternion to convert a rotation expressed in ENU body frame (ROS base_link) to
 * 			 a rotation expressed in NED body frame  (Z->Y->X convention)
 * Rotate 0 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis
 * 
 * NOTE: this quaternion is as valid as the quaternion representing the rotation from NED to ENU (quaternion ambiguity) on
 * 			the body frame
*/
template <typename T>
static const Eigen::Quaternion<T> ENU_NED_BODY_Q = Pegasus::Rotations::euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, 0.0));

/**
 * @brief Static quaternion needed for rotating vectors in body frames between ENU and NED
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED)
 * Fto Forward, Left, Up (body frame in ENU).
 */
template <typename T>
static const Eigen::Quaternion<T> BODY_ENU_NED_Q = Pegasus::Rotations::euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, 0.0));

/**
 * @brief Static affine matrix to roate vectors ENU (or NED) -> NED (or ENU) expressed in body frame
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED)
 * Fto Forward, Left, Up (body frame in ENU).
 */
template <typename T>
static const Eigen::Transform<T, 3, Eigen::Affine> BODY_ENU_NED_TF = Eigen::Transform<T, 3, Eigen::Affine>(BODY_ENU_NED_Q<T>);
//template <typename T>
//static const Eigen::Matrix<T, 3, 3> BODY_ENU_NED_AXIS = BODY_ENU_NED_Q<T>.toRotationMatrix();


/**
 * @brief Use reflections instead of rotations for NED <-> ENU transformation
 * to avoid NaN/Inf floating point pollution across different axes
 * since in NED <-> ENU the axes are perfectly aligned.
 */
static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_XY(Eigen::Vector3i(1, 0, 2));
template <typename T>
static const Eigen::DiagonalMatrix<T, 3> NED_ENU_REFLECTION_Z(1, 1, -1);


/**
 * @brief Transform a rotation (as a quaternion) from body expressed in ENU (or NED) to inertial frame 
 * 			to a similar rotation (as quaternion) from body expressed in NED (or ENU) to inertial frame.
 *
 * NOTE: Check http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
 * 	for more details behind these type of transformations towards obtaining rotations in different
 * 	frames of reference. 
 * 
 * @param q quaternion representing a rotation: body frame ENU (or NED) -> inertial frame (in arbitrary convention)
 * @return quaternion represeting a rotation: body frame NED (or ENU) -> inertial frame (in arbitrary convention)
 */
template <typename T>
inline Eigen::Quaternion<T> rot_body_rotation(const Eigen::Quaternion<T> &q) {
	return q * ENU_NED_BODY_Q<T>;
}

/**
 * @brief Transform a rotation (as a quaternion) from body to inertial frame expressed in ENU (or NED)
 * 			to a similar rotation (as quaternion) from body to inertial frame expressed in NED (or ENU)
 * 
 * NOTE: Check http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
 * 	for more details behind these type of transformations towards obtaining rotations in different
 * 	frames of reference.
 * 
 * @param q quaternion representing a rotation: body frame (in arbitrary convention) -> inertial frame ENU (or NED)
 * @return quaternion represeting a rotation: body frame (in arbitrary convention) -> inertial frame NED (or ENU)
 */
template <typename T>
inline Eigen::Quaternion<T> rot_inertial_rotation(const Eigen::Quaternion<T> &q) {
	return ENU_NED_INERTIAL_Q<T> * q;
}


/**
 * @brief Transform a rotation of a rigid body (as a quaternion) from body (ENU or NED) to inertial frame (ENU or NED)
 * 			to a similar rotation (as quaternion) from body (NED or ENU) to inertial frame (NED or ENU)
 * 
 * NOTE: This function is usefull to convert the attitude of a vehicle from "ROS" quaternion to a typicall literature 
 * quaternion (where both the body frame and inertial frames are in ENU). If you are converting a quaternion that expresses
 * the orientation of a sensor with respect to a rigid body's body frame (and not the inertial frame), then you DO NOT WANT TO USE THIS FUNCTION. 
 * Body-FRAME NED is not the same as INERTIAL-FRAME NED (this comes once again from the fact that in ned body
 * the x-y axis don't switch like in inertial frame) as explained in the documentation.
 * 
 * Essencial only use this if you are representing a body in inertial frame!
 * 
 * NOTE: Check http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
 * 	for more details behind these type of transformations towards obtaining rotations in different
 * 	frames of reference.
 * 
 * @param q quaternion representing a rotation: body frame (ENU or NED) -> inertial frame (ENU or NED)
 * @return quaternion representing a rotation: body frame (NED or ENU) -> inertial frame (NED or ENU)
 */
template <typename T>
inline Eigen::Quaternion<T> rot_body_to_inertial(const Eigen::Quaternion<T> &q) {
	return rot_inertial_rotation(rot_body_rotation(q));
}


/**
 * @brief Transform vector in ENU (or NED) to NED (or ENU), expressed in body-frame.
 * 	+PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED)
 * 	Fto Forward, Left, Up (body frame in ENU).
 * 
 * @param vec Vector expressed in body-frame (ENU or NED)
 * @return Vector expressed in body-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> transform_vect_body_enu_ned(const Eigen::Matrix<T,3,1> &vec) {
	return BODY_ENU_NED_TF<T> * vec;
}

/**
 * @brief Transform a vector in a given frame of reference to another frame of reference.
 * 
 * @param vec Vector expressed in the original frame of reference
 * @param q Quaternion that expresses the orientation of the original frame of reference with respect to the final frame of reference
 * @return Vector expressed in the new frame of reference
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> transform_vect_between_arbitrary_ref(const Eigen::Matrix<T, 3, 1> &vec, const Eigen::Quaternion<T> &q) {

	// Create an Affine3D transform with the rotation between the reference frames
	const Eigen::Transform<T, 3, Eigen::Affine> frame_conversion = Eigen::Transform<T, 3, Eigen::Affine>(q);
	return frame_conversion * vec;
}	

/**
 * @brief Transform vector in ENU (or NED) to NED (or ENU), expressed in inertial-frame.
 *  ENU <---> NED - Invert the Z axis and switch the XY axis
 * 
 * @param vec Vector expressed in inertial-frame (ENU or NED)
 * @return Vector expressed in inertial-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> transform_vect_inertial_enu_ned(const Eigen::Matrix<T,3,1> &vec) {
	return NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z<T> * vec);
}


/**
 * @brief Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in body-frame.
 * 	
 * NOTE: Check https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance for a detailed
 * 	explanation of the actual conversion proof for covariance matrices
 * 
 * @param cov_in Covariance matrix expressed in body-frame (ENU or NED)
 * @return Covariance matrix expressed in body-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> transform_cov3_body_enu_ned(const Eigen::Matrix<T, 3, 3> &cov_in) {
	return cov_in * BODY_ENU_NED_Q<T>;
}


/**
 * @brief Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in inertial-frame.
 * 
 * NOTE: Check https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance for a detailed
 * 	explanation of the actual conversion proof for covariance matrices
 * 
 * @param cov_in Covariance matrix expressed in inertial-frame (ENU or NED)
 * @return Covariance matrix expressed in inertial-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> transform_cov3_inertial_enu_ned(const Eigen::Matrix<T, 3, 3> &cov_in) {
	Eigen::Matrix<T, 3, 3> cov_out;

	cov_out = NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z<T> * cov_in * NED_ENU_REFLECTION_Z<T> ) *
        NED_ENU_REFLECTION_XY.transpose();
    
	return cov_out;
}

}}