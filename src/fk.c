#include <stdio.h>
#include <stdlib.h>

#include <cmink/fk.h>
#include <cmink/state.h>
#include <cmink/chain.h>
#include <cmink/geometry.h>
#include <cmink/ops.h>

se3_t fk(chain_t *chain, cstate_t state) {

  if (chain->dof != state.dof) {
    fprintf(stderr, "error: dof mismatch\n");
    fprintf(stderr, "       chain dof = %d\n", chain->dof);
    fprintf(stderr, "       state dof = %d\n", state.dof);
    exit(1);
  }

  se3_t T = se3_I();

  // T now holds transformation from
  // base link to base link (i.e. I)
  for (int i = 0; i < chain->dof; ++i) {
    T = se3_mul(T, jointT(state.q[i], chain->joints[i]));
  }

  return T;
}
