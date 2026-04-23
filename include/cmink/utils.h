/*
 * this file contains helper functions
 * to debug things in the lib
*/

#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>
#include <cmink/geometry3d.h>

#ifndef OUT_STREAM
# define OUT_STREAM stdout
#endif

/* display vec3_t */
void vec3_show(vec3_t v) {
  fprintf(OUT_STREAM,
    "vec3(%.3lf %.3lf %.3lf)\n",
    v.x, v.y, v.z);
}

/* display so3_t */
void so3_show(so3_t q) {
  fprintf(OUT_STREAM,
    "so3(%.3lf %c %.3lfi %c %.3lfj %c %.3lfk)\n",
     q.w,
     ((q.x >= 0) ? '+': '-'), fabs(q.x),
     ((q.y >= 0) ? '+': '-'), fabs(q.y),
     ((q.z >= 0) ? '+': '-'), fabs(q.z));
}

/* display so3_rpy_t */
void so3_rpy_show(so3_rpy_t rpy) {
  fprintf(OUT_STREAM,
    "so3_rpy(r=%.3lf p=%.3lf y=%.3lf)\n",
    rpy.r, rpy.p, rpy.y);
}

/* display axis_t */
void axis_show(axis_t a) {
  fprintf(OUT_STREAM,
    "axis(%.3lf %.3lf %.3lf)\n",
    a.x, a.y, a.z);
}

/* display se3_t */
void se3_show(se3_t T) {
  fprintf(OUT_STREAM,
    "se3(\n"
    "  t=vec3(%.3lf %.3lf %.3lf)\n"
    "  R=so3(%.3lf %c %.3lfi %c %.3lfj %c %.3lfk)\n"
    ")\n",
    T.t.x, T.t.y, T.t.z,
    T.R.w,
    ((T.R.x >= 0) ? '+': '-'), fabs(T.R.x),
    ((T.R.y >= 0) ? '+': '-'), fabs(T.R.y),
    ((T.R.z >= 0) ? '+': '-'), fabs(T.R.z));
}

/* display matrix3d_t */
void matrix3d_show(const matrix3d_t m) {
  fprintf(OUT_STREAM,
    "matrix3d([\n"
    "  [%.3lf %.3lf %.3lf]\n"
    "  [%.3lf %.3lf %.3lf]\n"
    "  [%.3lf %.3lf %.3lf]\n"
    "])\n",
    m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2]);
}

/* display matrix4d_t */
void matrix4d_show(const matrix4d_t m) {
  fprintf(OUT_STREAM,
    "matrix4d([\n"
    "  [%.3lf %.3lf %.3lf %.3lf]\n"
    "  [%.3lf %.3lf %.3lf %.3lf]\n"
    "  [%.3lf %.3lf %.3lf %.3lf]\n"
    "  [%.3lf %.3lf %.3lf %.3lf]\n"
    "])\n",
    m[0][0], m[0][1], m[0][2], m[0][3],
    m[1][0], m[1][1], m[1][2], m[1][3],
    m[2][0], m[2][1], m[2][2], m[2][3],
    m[3][0], m[3][1], m[3][2], m[3][3]);
}

#endif
