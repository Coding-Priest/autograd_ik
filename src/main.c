#include <stdio.h>
#include <stdlib.h>

#include <cmink/chain.h>
#include <cmink/geometry3d.h>

int main(int argc, char **argv) {

  ktree_t *tree = furdf2chain("piper.urdf");
  chain_t *chain = get_chain(tree, "base_link", "link7");

  for (int t = 0; t < chain->dof; ++t) {
    printf("%s\n", chain->joints[t].name);
  }

  free_tree(tree);
  free_chain(chain);
  return 0;
}
