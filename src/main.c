#include <stdio.h>
#include <stdlib.h>

#define OUT_STREAM stderr

#include <cmink/chain.h>
#include <cmink/utils.h>
#include <cmink/geometry3d.h>

int main(int argc, char **argv) {

  ktree_t *tree = furdf2chain("piper.urdf");
  for (int t = 0; t < tree->n_joint; ++t) {
    printf("joint name: %s\n", tree->joints[t].name);
    printf("joint origin: ");
    matrix4d_t m;
    se32homo(tree->joints[t].origin, m);
    matrix4d_show(m);
  }

  chain_t *chain = get_chain(tree, "base_link", "link7");
  for (int t = 0; t < chain->dof; ++t) {
    printf("%s\n", chain->joints[t].name);
  }

  free_tree(tree);
  free_chain(chain);
  return 0;
}
