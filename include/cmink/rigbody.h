/*
 * rigid body dynamics 
 * holders in this file,
 * it has the following 
 * structs
 *    - inertia -> moment of inertia tensor
 *    - inertial 
*/

#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <cmink/geometry3d.h>

typedef struct inertia  inertia_t;
typedef struct inertial inertial_t;

/* struct inertia
 * holds the moment of inertia tensor,
 * which is a symmetric 3x3 matrix
 * ixx ixy ixz
 * iyx iyy iyz
 * izx izy izz
*/
struct inertia {
  /* diagonal terms */
  double ixx; // = dm * (y*y + z*z)
  double iyy; // = dm * (x*x + z*z)
  double izz; // = dm * (x*x + y*y)

  /* coupling terms */ 
  double ixy; // = iyx = -dm * x * y
  double iyz; // = izy = -dm * y * z
  double izx; // = ixz = -dm * z * x
};

struct inertial {
  se3_t     origin;
  double    mass; // linear resistance
  inertia_t inertia; // rotational resistance
};

#endif
