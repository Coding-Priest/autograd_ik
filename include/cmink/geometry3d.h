/*
 * goemetry3d contains the required,
 * lie groups and transformation 
 * entities
*/

#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

typedef struct vec3    vec3_t;
typedef struct so3     so3_t;
typedef struct so3rpy  so3_rpy_t;
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

/* rpy to quat */
so3_t rpy2quat(so3_rpy_t rpy);

/* quat to rpy */
so3_rpy_t quat2rpy(so3 q);

/* mat to quat */
so3_t mat2quat(matrix3d_t mat);

/* quat to mat */
matrix3d_t quat2mat(so3_t q);

/* mat to rpy */
/* rpy to mat */

/* axis norm */
/* quat norm */

#endif
