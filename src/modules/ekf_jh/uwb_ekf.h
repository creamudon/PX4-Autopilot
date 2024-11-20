/*
 * MedianFilter.h
 *
 *  Created on: May 19, 2018
 *      Author: alexandru.bogdan
 */

#ifndef UWB_EKF_H_
#define UWB_EKF_H_

#ifdef __cplusplus
extern "C" {
#endif

void UWBEKF_vel_2UWB_with_Input_init(void);

void UWBEKF_vel_2UWB_with_Input(double z[], double u[], double dt,
                                double *pos_x, double *pos_y, double *pos_z,
                                double *vel_y);

#ifdef __cplusplus
}
#endif
#endif
