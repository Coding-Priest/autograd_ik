/*
 * operations on matrices 
 * and other lie groups
*/

#ifndef OPS_H
#define OPS_H

#include <cmink/geometry.h>

/* vector addition, used for translation operation */
vec3_t vec3_add(vec3_t v1, vec3_t v2);

/* 
 * vector dot to find angles between two vector 
 * note: it returns a scalar
*/
double vec3_dot(vec3_t v1, vec3_t v2);

/* vector cross product */
vec3_t vec3_cross(vec3_t v1, vec3_t v2);

/* rotate a vector using a quaternion */
vec3_t vec3_rot(vec3_t v, so3_t q);

/* 
 * quaternion multiplication
 * => applying q2 on q1
*/
so3_t so3_mul(so3_t q1, so3_t q2);

/* se3 multiplication */
se3_t se3_mul(se3_t T1, se3_t T2);

/* constant methods */
vec3_t vec3_O();
so3_t  so3_I();
se3_t  se3_I();

/* rpy to quat
 * so3_rpy_t rpy = {roll, pitch, yaw};
 * so3_t q = rpy2quat(rpy);
*/
so3_t rpy2quat(so3_rpy_t rpy);

/* quat to rpy
 * so3_t q = {...};
 * so3_rpy_t rpy = quat2rpy(q);
*/
so3_rpy_t quat2rpy(so3_t q);

/* mat to quat
 * matrix3d_t R = { ... };
 * so3_t q = mat2quat(R);
*/
so3_t mat2quat(const matrix3d_t mat);

/* quat to mat
 * so3_t q = {w, x, y, z};
 * matrix3d_t R;
 * quat2mat(q, R);
*/
void quat2mat(so3_t q, matrix3d_t mat);

/* mat to rpy
 * matrix3d_t R = { ... };
 * so3_rpy_t rpy = mat2rpy(R);
*/
so3_rpy_t mat2rpy(const matrix3d_t mat);

/* rpy to mat
 * so3_rpy_t rpy = {roll, pitch, yaw};
 * matrix3d_t R;
 * rpy2mat(rpy, R);
*/
void rpy2mat(so3_rpy_t rpy, matrix3d_t mat);

/* homogeneous transformation matrix to se3 */
se3_t homo2se3(const matrix4d_t mat);

/* se3 to homogeneous transformation matrix */
void se32homo(se3_t se3, matrix4d_t mat);

/* 
 * converts joint revolution angle
 * and joint revolution axis to a
 * quaternion
 * note: this assumes the axis is in normalized
 *       form
*/
so3_t axis2rot(double angle, axis_t axis);

/* axis norm */
void anorm(axis_t *axis);

/* quat norm */
void qnorm(so3_t *q);

#endif
