/*
 * forward kinematics 
*/

#ifndef FK_H
#define FK_H

#include <cmink/goemetry3d.h>
#include <cmink/state.h>
#include <cmink/chain.h>

/* 
 * forward transformations is a list of homogeneous
 * transformation matrices parent -> joint -> child
 * is 3 transfomations 
 * parent origin -> joint origin => translation
 * joint origin revolution       => rotation
 * joint origin -> child origin  => translation
 * number of matrices = dof
*/
se3_t fk(chain_t *chain, cstate_t q);

#endif
