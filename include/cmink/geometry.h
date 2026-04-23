/*
 * goemetry3d contains the required,
 * lie groups and transformation 
 * entities
*/

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

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

#endif
