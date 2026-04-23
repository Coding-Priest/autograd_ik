#include <math.h>
#include <cmink/geometry.h>
#include <cmink/ops.h>

vec3_t vec3_add(vec3_t v1, vec3_t v2) {
  return (vec3_t) {
    .x = v1.x + v2.x,
    .y = v1.y + v2.y,
    .z = v1.z + v2.z,
  };
}

double vec3_dot(vec3_t v1, vec3_t v2) {
  return v1.x * v2.x + 
         v1.y * v2.y +
         v1.z * v2.z;
}

vec3_t vec3_cross(vec3_t v1, vec3_t v2) {
  return (vec3_t) {
    .x = v1.y * v2.z - v1.z * v2.y,
    .y = v1.z * v2.x - v1.x * v2.z,
    .z = v1.x * v2.y - v1.y * v2.x,
  };
}

vec3_t vec3_rot(vec3_t v, so3_t q) {
  return (vec3_t) {
    .x = v.x * q.w + v.y * q.z - v.z * q.y,
    .y = v.y * q.w + v.x * q.y - v.z * q.x,
    .z = v.z * q.w + v.x * q.y - v.y * q.x
  };
}

so3_t so3_mul(so3_t q1, so3_t q2) {
  return (so3_t) {
    .w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    .x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    .y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    .z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
  };
}

se3_t se3_mul(se3_t T1, se3_t T2) {
  // T1 * T2 = [R1 R2,  R1 * t2 + t1]
  //           [0    ,  1           ]
  vec3_t rotated_t2 = vec3_rot(T2.t, T1.R);
  return (se3_t) {
    .t = vec3_add(T1.t, rotated_t2),
    .R = so3_mul(T1.R, T2.R),
  };
}

vec3_t vec3_O() {
  return (vec3_t) {
    .x = 0,
    .y = 0,
    .z = 0
  };
}

so3_t so3_I() {
  return (so3_t) {
    .w = 1,
    .x = 0,
    .y = 0,
    .z = 0
  };
}

se3_t se3_I(){
  se3_t T;
  T.t = vec3_O();
  T.R = so3_I();
  return T;
}

so3_t rpy2quat(so3_rpy_t rpy) {
  so3_t q;

  double cr = cos(rpy.r * 0.5);
  double sr = sin(rpy.r * 0.5);

  double cp = cos(rpy.p * 0.5);
  double sp = sin(rpy.p * 0.5);

  double cy = cos(rpy.y * 0.5);
  double sy = sin(rpy.y * 0.5);

  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = cy * sp * cr + sy * cp * sr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

so3_rpy_t quat2rpy(so3_t q) {
  so3_rpy_t rpy;

  double w = q.w;
  double x = q.x;
  double y = q.y;
  double z = q.z;

  double sr = 2.0 * (w * x + y * z);
  double cr = 1.0 - 2.0 * (x * x + y * y);
  rpy.r = atan2(sr, cr);

  double sp = 2.0 * (w * y - z * x);

  if (sp >= 1.0) rpy.p =  M_PI_2;
  else if (sp <= -1.0) rpy.p = -M_PI_2;
  else rpy.p = asin(sp);

  double sy = 2.0 * (w * z + x * y);
  double cy = 1.0 - 2.0 * (y * y + z * z);
  rpy.y = atan2(sy, cy);

  return rpy;
}


// copied this from an llm
// I don't understand this at all
so3_t mat2quat(const matrix3d_t m) {
  so3_t q;
  double t = m[0][0] + m[1][1] + m[2][2];

  if (t > 0.0) {
    double s = sqrt(t + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (m[2][1] - m[1][2]) / s;
    q.y = (m[0][2] - m[2][0]) / s;
    q.z = (m[1][0] - m[0][1]) / s;
  }else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    double s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q.w = (m[2][1] - m[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (m[0][1] + m[1][0]) / s;
    q.z = (m[0][2] + m[2][0]) / s;
  }else if (m[1][1] > m[2][2]) {
    double s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    q.w = (m[0][2] - m[2][0]) / s;
    q.x = (m[0][1] + m[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (m[1][2] + m[2][1]) / s;
  }else {
    double s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    q.w = (m[1][0] - m[0][1]) / s;
    q.x = (m[0][2] + m[2][0]) / s;
    q.y = (m[1][2] + m[2][1]) / s;
    q.z = 0.25 * s;
  }

  return q;
}

void quat2mat(so3_t q, matrix3d_t mat) {
  double x2 = q.x * q.x;
  double y2 = q.y * q.y;
  double z2 = q.z * q.z;

  double wx = q.w * q.x;
  double wy = q.w * q.y;
  double wz = q.w * q.z;

  double xy = q.x * q.y;
  double xz = q.x * q.z;
  double yz = q.y * q.z;

  mat[0][0] = 1.0 - 2.0 * (y2 + z2);
  mat[0][1] = 2.0 * (xy - wz);
  mat[0][2] = 2.0 * (xz + wy);

  mat[1][0] = 2.0 * (xy + wz);
  mat[1][1] = 1.0 - 2.0 * (x2 + z2);
  mat[1][2] = 2.0 * (yz - wx);

  mat[2][0] = 2.0 * (xz - wy);
  mat[2][1] = 2.0 * (yz + wx);
  mat[2][2] = 1.0 - 2.0 * (x2 + y2);
}

so3_rpy_t mat2rpy(const matrix3d_t m) {
  so3_rpy_t rpy;
  double sp = -m[2][0];

  if (sp >= 1.0) rpy.p =  M_PI_2;
  else if (sp <= -1.0) rpy.p = -M_PI_2;
  else rpy.p = asin(sp);

  double cp = cos(rpy.p);

  if (fabs(cp) > 1e-8) {
    rpy.r = atan2(m[2][1] / cp, m[2][2] / cp);
    rpy.y = atan2(m[1][0] / cp, m[0][0] / cp);
  }else {
    rpy.r = 0.0;
    rpy.y = atan2(-m[0][1], m[1][1]);
  }

  return rpy;
}

void rpy2mat(so3_rpy_t rpy, matrix3d_t m) {
  double cr = cos(rpy.r);
  double sr = sin(rpy.r);

  double cp = cos(rpy.p);
  double sp = sin(rpy.p);

  double cy = cos(rpy.y);
  double sy = sin(rpy.y);

  m[0][0] = cy * cp;
  m[0][1] = cy * sp * sr - sy * cr;
  m[0][2] = cy * sp * cr + sy * sr;

  m[1][0] = sy * cp;
  m[1][1] = sy * sp * sr + cy * cr;
  m[1][2] = sy * sp * cr - cy * sr;

  m[2][0] = -sp;
  m[2][1] = cp * sr;
  m[2][2] = cp * cr;
}

se3_t homo2se3(const matrix4d_t mat) {
  se3_t T;

  T.t.x = mat[0][3];
  T.t.y = mat[1][3];
  T.t.z = mat[2][3];

  matrix3d_t R;
  R[0][0] = mat[0][0]; R[0][1] = mat[0][1]; R[0][2] = mat[0][2];
  R[1][0] = mat[1][0]; R[1][1] = mat[1][1]; R[1][2] = mat[1][2];
  R[2][0] = mat[2][0]; R[2][1] = mat[2][1]; R[2][2] = mat[2][2];

  T.R = mat2quat(R);
  return T;
}

void se32homo(se3_t T, matrix4d_t mat) {
  matrix3d_t R;
  quat2mat(T.R, R);

  mat[0][0] = R[0][0]; mat[0][1] = R[0][1]; mat[0][2] = R[0][2];
  mat[1][0] = R[1][0]; mat[1][1] = R[1][1]; mat[1][2] = R[1][2];
  mat[2][0] = R[2][0]; mat[2][1] = R[2][1]; mat[2][2] = R[2][2];

  mat[0][3] = T.t.x;
  mat[1][3] = T.t.y;
  mat[2][3] = T.t.z;

  mat[3][0] = 0.0;
  mat[3][1] = 0.0;
  mat[3][2] = 0.0;
  mat[3][3] = 1.0;
}

so3_t axis2rot(double angle, axis_t axis) {
  so3_t q;
  q.w = cos(angle);
  q.x = sin(angle) * axis.x;
  q.y = sin(angle) * axis.y;
  q.z = sin(angle) * axis.z;
  return q;
}

void anorm(axis_t *axis) {
  double den = sqrt(axis->x * axis->x + 
                    axis->y * axis->y + 
                    axis->z * axis->z);
  if (den == 0) return ;
  axis->x /= den;
  axis->y /= den;
  axis->z /= den;
}

void qnorm(so3_t *q) {
  double den = sqrt(q->w + q->w +
                    q->x * q->x + 
                    q->y * q->y + 
                    q->z * q->z);
  if (den == 0) return ;
  q->w /= den;
  q->x /= den;
  q->y /= den;
  q->z /= den;
}
