#include <stdio.h>
#include <stdlib.h>

#define OUT_STREAM stderr

#include <cmink/chain.h>
#include <cmink/utils.h>
#include <cmink/geometry.h>
#include <cmink/ops.h>
#include <cmink/state.h>
#include <cmink/fk.h>

int main(int argc, char **argv) {

  ktree_t *tree = furdf2chain("piper.urdf");

  chain_t *chain = get_chain(tree, "base_link", "link7");

  cstate_t state;
  state.dof = chain->dof;
  state.q = malloc(sizeof(double) * 8);

  printf("chain fixed transforms\n");
  for (int t = 0; t < chain->dof; ++t) {
    matrix4d_t T;
    se32homo(jointT(0.0, chain->joints[t]), T);
    matrix4d_show(T);
  }

  se3_t fk_pose = fk(chain, state);
  matrix4d_t T;
  se32homo(fk_pose, T);
  printf("solution: \n");
  matrix4d_show(T);

  free_tree(tree);
  free_chain(chain);
  return 0;
}
