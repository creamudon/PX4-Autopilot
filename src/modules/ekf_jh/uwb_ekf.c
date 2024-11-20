#include "uwb_ekf.h"
#include <math.h>
#include <string.h>

#define EPSILON 0.000001


static int isInitialized_UWBEKF_vel_2UWB_with_Input=0;

/* Variable Definitions */
static double Q[16];

static double R[25];

static double x[4];

static double P[16];

static double p_uwb_1[3];

static double p_uwb_2[3];

/* Function Definitions */
/*
 * Arguments    : const double z[5]
 *                const double u[3]
 *                double dt
 *                double *pos_x
 *                double *pos_y
 *                double *pos_z
 *                double *vel_y
 * Return Type  : void
 */



void UWBEKF_vel_2UWB_with_Input(double z[], double u[], double dt,
                                double *pos_x, double *pos_y, double *pos_z,
                                double *vel_y)
{
  double b_x[25];
  double y[25];
  double H[20];
  double K[20];
  double b_H[20];
  double A[16];
  double Pp[16];
  double b_A[16];
  double b_dt[12];
  double b_z[5];
  double xp[4];
  double H_tmp;
  double a_tmp;
  double b_a_tmp;
  double c_a_tmp;
  double d_a_tmp;
  double s;
  double smax;
  int b_i;
  int b_tmp;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  int kAcol;
  int mmj_tmp;
  int x_tmp;
  signed char ipiv[5];
  signed char p[5];

  if (!isInitialized_UWBEKF_vel_2UWB_with_Input) {
    UWBEKF_vel_2UWB_with_Input_init();
    isInitialized_UWBEKF_vel_2UWB_with_Input=1;
  }

  /*  persistent firstRun */
  A[1] = 0.0;
  A[5] = 1.0;
  A[9] = 0.0;
  A[13] = dt;
  smax = x[0] - p_uwb_1[0];
  s = x[1] - p_uwb_1[1];
  a_tmp = x[2] - p_uwb_1[2];
  b_a_tmp = x[0] - p_uwb_2[0];
  c_a_tmp = x[1] - p_uwb_2[1];
  d_a_tmp = x[2] - p_uwb_2[2];

  H_tmp = sqrt((smax * smax + s * s) + a_tmp * a_tmp);

  H[1] = smax / H_tmp;
  H[6] = s / H_tmp;
  H[11] = a_tmp / H_tmp;
  H[16] = 0.0;
  H_tmp = sqrt((b_a_tmp * b_a_tmp + c_a_tmp * c_a_tmp) + d_a_tmp * d_a_tmp);
  H[2] = b_a_tmp / H_tmp;
  H[7] = c_a_tmp / H_tmp;
  H[12] = d_a_tmp / H_tmp;
  H[17] = 0.0;
  A[0] = 1.0;
  A[2] = 0.0;
  A[3] = 0.0;
  H[0] = 1.0;
  H[3] = 0.0;
  H[4] = 0.0;
  A[4] = 0.0;
  A[6] = 0.0;
  A[7] = 0.0;
  H[5] = 0.0;
  H[8] = 0.0;
  H[9] = 0.0;
  A[8] = 0.0;
  A[10] = 1.0;
  A[11] = 0.0;
  H[10] = 0.0;
  H[13] = 1.0;
  H[14] = 0.0;
  A[12] = 0.0;
  A[14] = 0.0;
  A[15] = 1.0;
  H[15] = 0.0;
  H[18] = 0.0;
  H[19] = 1.0;
  smax = dt * dt / 2.0;
  b_dt[0] = smax;
  b_dt[4] = 0.0;
  b_dt[8] = 0.0;
  b_dt[1] = 0.0;
  b_dt[5] = smax;
  b_dt[9] = 0.0;
  b_dt[2] = 0.0;
  b_dt[6] = 0.0;
  b_dt[10] = smax;
  b_dt[3] = 0.0;
  b_dt[7] = dt;
  b_dt[11] = 0.0;

  for (i = 0; i < 4; i++) {
    smax = A[i + 4];
    s = A[i + 8];
    a_tmp = A[i + 12];
    xp[i] = (((A[i] * x[0] + smax * x[1]) + s * x[2]) + a_tmp * x[3]) +
            ((b_dt[i] * u[0] + b_dt[i + 4] * u[1]) + b_dt[i + 8] * u[2]);
    for (jp1j = 0; jp1j < 4; jp1j++) {
      kAcol = jp1j << 2;
      b_A[i + kAcol] =
          ((A[i] * P[kAcol] + smax * P[kAcol + 1]) + s * P[kAcol + 2]) +
          a_tmp * P[kAcol + 3];
    }
    smax = b_A[i];
    s = b_A[i + 4];
    a_tmp = b_A[i + 8];
    b_a_tmp = b_A[i + 12];
    for (jp1j = 0; jp1j < 4; jp1j++) {
      jA = i + (jp1j << 2);
      Pp[jA] = (((smax * A[jp1j] + s * A[jp1j + 4]) + a_tmp * A[jp1j + 8]) +
                b_a_tmp * A[jp1j + 12]) +
               Q[jA];
    }
  }

  for (i = 0; i < 5; i++) {
    for (jp1j = 0; jp1j < 4; jp1j++) {
      jA = i + 5 * jp1j;
      K[jp1j + (i << 2)] = H[jA];
      kAcol = jp1j << 2;
      b_H[jA] = ((H[i] * Pp[kAcol] + H[i + 5] * Pp[kAcol + 1]) +
                 H[i + 10] * Pp[kAcol + 2]) +
                H[i + 15] * Pp[kAcol + 3];
    }
  }

  for (i = 0; i < 5; i++) {
    smax = b_H[i];
    s = b_H[i + 5];
    a_tmp = b_H[i + 10];
    b_a_tmp = b_H[i + 15];
    for (jp1j = 0; jp1j < 5; jp1j++) {
      kAcol = jp1j << 2;
      x_tmp = i + 5 * jp1j;
      b_x[x_tmp] =
          (((smax * K[kAcol] + s * K[kAcol + 1]) + a_tmp * K[kAcol + 2]) +
           b_a_tmp * K[kAcol + 3]) +
          R[x_tmp];
    }
  }

  memset(&y[0], 0, 25U * sizeof(double));
  for (i = 0; i < 5; i++) {
    ipiv[i] = (signed char)(i + 1);
  }

  for (j = 0; j < 4; j++) {
    mmj_tmp = 3 - j;
    b_tmp = j * 6;
    jp1j = b_tmp + 2;
    jA = 5 - j;
    kAcol = 0;
    smax = fabs(b_x[b_tmp]);
    for (k = 2; k <= jA; k++) {
      s = fabs(b_x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }

    if ( (double)fabs(b_x[b_tmp + kAcol]) > EPSILON) {
      if (kAcol != 0) {
        jA = j + kAcol;
        ipiv[j] = (signed char)(jA + 1);
        for (k = 0; k < 5; k++) {
          kAcol = j + k * 5;
          smax = b_x[kAcol];
          x_tmp = jA + k * 5;
          b_x[kAcol] = b_x[x_tmp];
          b_x[x_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 5;
      for (b_i = jp1j; b_i <= i; b_i++) {
        b_x[b_i - 1] /= b_x[b_tmp];
      }
    }

    jA = b_tmp;

    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = b_x[(b_tmp + kAcol * 5) + 5];
      if ( fabs(smax) > EPSILON) {
        i = jA + 7;
        jp1j = (jA - j) + 10;
        for (x_tmp = i; x_tmp <= jp1j; x_tmp++) {
          b_x[x_tmp - 1] += b_x[((b_tmp + x_tmp) - jA) - 6] * -smax;
        }
      }
      jA += 5;
    }
  }

  for (i = 0; i < 5; i++) {
    p[i] = (signed char)(i + 1);
  }

  if (ipiv[0] > 1) {
    jA = p[ipiv[0] - 1];
    p[ipiv[0] - 1] = p[0];
    p[0] = (signed char)jA;
  }

  if (ipiv[1] > 2) {
    jA = p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = (signed char)jA;
  }

  if (ipiv[2] > 3) {
    jA = p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = (signed char)jA;
  }
  if (ipiv[3] > 4) {
    jA = p[ipiv[3] - 1];
    p[ipiv[3] - 1] = p[3];
    p[3] = (signed char)jA;
  }

  for (k = 0; k < 5; k++) {
    x_tmp = 5 * (p[k] - 1);
    y[k + x_tmp] = 1.0;
    for (j = k + 1; j < 6; j++) {
      i = (j + x_tmp) - 1;
      if (fabs(y[i]) > EPSILON) {
        jp1j = j + 1;
        for (b_i = jp1j; b_i < 6; b_i++) {
          jA = (b_i + x_tmp) - 1;
          y[jA] -= y[i] * b_x[(b_i + 5 * (j - 1)) - 1];
        }
      }
    }
  }

  for (j = 0; j < 5; j++) {
    jA = 5 * j;
    for (k = 4; k >= 0; k--) {
      kAcol = 5 * k;
      i = k + jA;
      smax = y[i];
      if (fabs(smax) > EPSILON) {
        y[i] = smax / b_x[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          x_tmp = b_i + jA;
          y[x_tmp] -= y[i] * b_x[b_i + kAcol];
        }
      }
    }
  }

  for (i = 0; i < 4; i++) {
    smax = Pp[i];
    s = Pp[i + 4];
    a_tmp = Pp[i + 8];
    b_a_tmp = Pp[i + 12];
    for (jp1j = 0; jp1j < 5; jp1j++) {
      kAcol = jp1j << 2;
      b_H[i + kAcol] =
          ((smax * K[kAcol] + s * K[kAcol + 1]) + a_tmp * K[kAcol + 2]) +
          b_a_tmp * K[kAcol + 3];
    }
  }

  smax = xp[0] - p_uwb_1[0];
  s = xp[1] - p_uwb_1[1];
  a_tmp = xp[2] - p_uwb_1[2];
  b_a_tmp = xp[0] - p_uwb_2[0];
  c_a_tmp = xp[1] - p_uwb_2[1];
  d_a_tmp = xp[2] - p_uwb_2[2];
  b_z[0] = z[0] - xp[0];
  b_z[1] = z[1] - sqrt((smax * smax + s * s) + a_tmp * a_tmp);
  b_z[2] =
      z[2] - sqrt((b_a_tmp * b_a_tmp + c_a_tmp * c_a_tmp) + d_a_tmp * d_a_tmp);
  b_z[3] = z[3] - xp[2];
  b_z[4] = z[4] - xp[3];
  for (i = 0; i < 4; i++) {
    smax = 0.0;
    for (jp1j = 0; jp1j < 5; jp1j++) {
      s = 0.0;
      for (kAcol = 0; kAcol < 5; kAcol++) {
        s += b_H[i + (kAcol << 2)] * y[kAcol + 5 * jp1j];
      }
      K[i + (jp1j << 2)] = s;
      smax += s * b_z[jp1j];
    }
    x[i] = xp[i] + smax;
    for (jp1j = 0; jp1j < 4; jp1j++) {
      smax = 0.0;
      for (kAcol = 0; kAcol < 5; kAcol++) {
        smax += K[i + (kAcol << 2)] * H[kAcol + 5 * jp1j];
      }
      A[i + (jp1j << 2)] = smax;
    }
    smax = A[i];
    s = A[i + 4];
    a_tmp = A[i + 8];
    b_a_tmp = A[i + 12];
    for (jp1j = 0; jp1j < 4; jp1j++) {
      kAcol = jp1j << 2;
      jA = i + kAcol;
      P[jA] =
          Pp[jA] -
          (((smax * Pp[kAcol] + s * Pp[kAcol + 1]) + a_tmp * Pp[kAcol + 2]) +
           b_a_tmp * Pp[kAcol + 3]);
    }
  }
  *pos_x = x[0];
  *pos_y = x[1];
  *pos_z = x[2];
  *vel_y = x[3];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void UWBEKF_vel_2UWB_with_Input_init(void)
{
  static const double dv1[25] = {
      0.012, 0.0, 0.0, 0.0, 0.0, 0.0,   1.03, 0.0, 0.0, 0.0, 0.0, 0.0,  1.03,
      0.0,   0.0, 0.0, 0.0, 0.0, 0.012, 0.0,  0.0, 0.0, 0.0, 0.0, 100.0};
  static const double dv[16] = {0.01, 0.0, 0.0,  0.0, 0.0, 0.001, 0.0, 0.0,
                                0.0,  0.0, 0.01, 0.0, 0.0, 0.0,   0.0, 0.01};
  static const signed char iv[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
  int i;
  memcpy(&Q[0], &dv[0], 16U * sizeof(double));
  memcpy(&R[0], &dv1[0], 25U * sizeof(double));
  for (i = 0; i < 16; i++) {
    P[i] = iv[i];
  }
  p_uwb_1[0] = -2.0;
  p_uwb_2[0] = -2.0;
  p_uwb_1[1] = 0.0;
  p_uwb_2[1] = 2.0;
  p_uwb_1[2] = 0.46;
  p_uwb_2[2] = 0.46;
  /*      Q=[ 0.1 0  0 0; */
  /*          0 0.00005  0 0 ; */
  /*          0 0 0.00005 0  */
  /*          0 0 0 0.01 ]; */
  /*   */
  /*      R=[0.00012  0  0 0 0 */
  /*         0  0.001  0 0 0 */
  /*         0   0   0.0009 0 0 */
  /*         0  0  0 0.00012 0 */
  /*         0 0  0  0  10]; */
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  /*      p_uwb=[-2.73,1.5,-0.35]'; */
  /*      p_uwb_2=[-2.72,3,0.75]'; */
  /*      p_uwb_1=[-2.72,0,0.75]'; */
  /*      firstRun = 1; */
}

/*
 * File trailer for UWBEKF_vel_2UWB_with_Input.c
 *
 * [EOF]
 */
