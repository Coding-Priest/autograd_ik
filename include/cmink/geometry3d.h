/*
 * goemetry3d contains the required,
 * lie groups and transformation 
 * entities
*/

#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

typedef struct vec3 vec3_t;
typedef struct so3  so3_t;
typedef struct se3  se3_t;
typedef struct axis axis_t;

typedef double matrix3d_t[3][3];
typedef double matrix4d_t[4][4];

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

#endif
