/*
 * Bdot.h
 *
 *  Created on: Sep 1, 2024
 *      Author: Anjali
 */

#ifndef INC_BDOT_H_
#define INC_BDOT_H_

#include "estimator_main.h"
#include "IMU.h"

//Constants

#define DT 0.05  //sampling interval: duration between consecutive sensor readings
#define Kp 0.1f  //proportional gain for B-Dot algorithm
#define ERROR_THRESHOLD 0.2f //threshold angular velocity in rad/s

#define MAX_MOMENT_MTQ 0.2f	//max moment for MTQ in Am2
#define MAX_DUTY_CYCLE 0.8f //Maximum allowed duty cycle
void CalAttitudeError(sat_att_combined *pcombined_sat_att);
void CalTorque(imu_filter *pfilt_att, lsm9ds1_t *pBdata, sat_att_combined *pcombined_sat_att);
void takedecision(sat_att_combined *pcombined_sat_att, imu_filter *pfilt_att,
		lsm9ds1_t *pBdata);
void EN5V(void);
void DISABLE5V(void);
void SET_PWM_DUTY_CYCLE_Y(uint8_t Dy);
void SET_PWM_DUTY_CYCLE_Z(uint8_t Dz);

extern float mx;
extern float my;
extern float mz;

extern imu_filter filt_imu;

#endif /* INC_BDOT_H_ */
