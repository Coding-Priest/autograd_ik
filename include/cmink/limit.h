/*
 * file to store the limit
 * struct
*/

#ifndef LIMIT_H
#define LIMIT_H

typedef struct limit limit_t;

struct limit {
  double lower;
  double upper;
  double effort;
  double velocity;
};

#endif
