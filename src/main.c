#include <stdio.h>
#include <mujoco/mujoco.h>

int main(int argc, char **argv) {
  printf("successfully linked against MuJoCo version: %s\n", mj_versionString());
  return 0;
}
