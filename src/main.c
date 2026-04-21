#include <stdio.h>

#include <cmink/chain.h>

int main(int argc, char **argv) {

  ktree_t tree;
  char *emsg;
  int v = valid(&tree, emsg);
  printf("%d\n", v);

  return 0;
}
