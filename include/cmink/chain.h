/*
 * kinematic chain elements, 
 * contains link and joint
 * definitions.
*/

#ifndef CHAIN_H
#define CHAIN_H

#include <cmink/geometry3d.h>
#include <cmink/rigbody.h>
#include <cmink/limit.h>

#include <string.h>
#include <stdint.h>

typedef struct link  link_t;
typedef struct joint joint_t;
typedef struct ktree ktree_t;

enum joint_type {
  FIXED,
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  FLOATING,
  PLANAR
};

struct link {
  char *name;
  char *vis_mesh; 
  char *col_mesh;
  se3_t vis_pose;
  se3_t col_pose;
  inertial_t inertial;
};

struct joint {
  char *name;
  enum joint_type type;
  se3_t origin;
  link_t *parent;
  link_t *child;
  axis_t axis;
  limit_t limit;
};

struct ktree {
  char *name;
  
  uint32_t n_link;
  uint32_t n_joint;

  link_t  *links;
  joint_t *joints;
};

#endif
