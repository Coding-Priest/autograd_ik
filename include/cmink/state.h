/*
 * file holds structs about
 * the state of a robot
*/

#ifndef STATE_H
#define STATE_H

#include <stdint.h>

typedef struct cstate cstate_t;
typedef struct tstate tstate_t;

/* configuration state (a.k.a. joint angles) */
struct cstate {
  uint32_t dof;
  double *q; // holds the joint angles
};

/* tangential state (a.k.a. joint velocity) */
struct tstate {
  uint32_t dof;
  double *q_dot;
};

#endif
