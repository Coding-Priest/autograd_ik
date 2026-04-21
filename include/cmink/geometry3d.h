/*
 * goemetry3d contains the required,
 * lie groups and transformation 
 * entities
*/

#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

typedef struct vec3    vec3_t;
typedef struct so3     so3_t;
typedef struct so3_rpy so3_rpy_t;
typedef struct se3     se3_t;
typedef struct axis    axis_t;

typedef double matrix3d_t[3][3];
typedef double matrix4d_t[4][4];

typedef matrix3d_t so3_mat_t;
typedef matrix4d_t se3_mat_t;

struct vec3 {
  double x;
  double y;
  double z;
};

// note: make sure things are l2 normalized
// ps  : quaternions >> matrices
struct so3 {
  double w;
  double x;
  double y;
  double z;
};

// SO(3) but in terms of rpy
// follows the body 3-2-1 
// convension
struct so3_rpy {
  double r;
  double p;
  double y;
};

// SE(3) is a lie group containing
// translation and rotation
struct se3 {
  vec3_t t;
  so3_t  R;
};

// same as point, but l2 normalized
struct axis {
  double x;
  double y;
  double z;
};

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

/* axis norm */
void anorm(axis_t *axis);

/* quat norm */
void qnorm(so3_t *q);

#endif
