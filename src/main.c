#include <stdio.h>

#include <cmink/chain.h>
#include <cmink/geometry3d.h>

int main(int argc, char **argv) {

  so3_t q = {0.707, 0.707, 0, 0};
  so3_rpy_t rpy = quat2rpy(q);

  printf("%lf %lf %lf\n", rpy.r, rpy.p, rpy.y);

//   ktree_t *tree = furdf2chain("piper.urdf");
//   printf("robot name: %s\n", tree->name);
//   printf("number of links: %d\n", tree->n_link);
//   printf("number of joints: %d\n", tree->n_joint);
// 
//   for (int t = 0; t < tree->n_link; ++t) {
//     printf("link name: %s\n", tree->links[t].name);
//     printf("mesh file: %s\n", tree->links[t].vis_mesh);
//   }

  return 0;
}
