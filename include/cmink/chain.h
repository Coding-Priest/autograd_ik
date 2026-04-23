/*
 * kinematic chain elements, 
 * contains link and joint
 * definitions.
*/

#ifndef CHAIN_H
#define CHAIN_H

#include <stdint.h>
#include <stdbool.h>

#include <cmink/geometry3d.h>
#include <cmink/rigbody.h>
#include <cmink/limit.h>

typedef struct link  link_t;
typedef struct joint joint_t;
typedef struct ktree ktree_t;
typedef struct chain chain_t;

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
  se3_t vis_pose;
  char *col_mesh;
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
  link_t  *links;  // list of links
  joint_t *joints; // list of joints
  link_t *root;
};

struct chain {
  int dof;

  link_t *base;
  link_t *tip;

  /* 
   * all the joints in the chain
   * in order 
  */
  joint_t *joints;
};

/* 
 * checks if the kinematic tree is valid
 * - find the root of the tree
 * - checks if tree is DAG 
*/
bool valid(ktree_t *tree);

/* loads the chain from a urdf string */
ktree_t *urdf2chain(char *urdf_string);

/* loads the chain from a urdf file */
ktree_t *furdf2chain(char *urdf_file);

/* free the tree safely to avoid memory leaks */
void free_tree(ktree_t *tree);

/* 
 * construction of chain from
 * base and tip link
 * assumes height(base) < hight(tip)
 * TODO: remove this assumption ^
*/
chain_t *get_chain(
  ktree_t *tree, char *base, char *tip);

/* free the chain safely to avoid memory leaks */
void free_chain(chain_t *chain);

#endif
