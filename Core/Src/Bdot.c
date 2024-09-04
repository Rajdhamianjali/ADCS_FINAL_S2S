/*
 * Bdot.c
 *
 *  Created on: Sep 1, 2024
 *      Author: Anjali
 */

#include "Bdot.h"
#include "variables.h"
#include "USER_FUNCTIONS.h"

float attitude_error[3];
float Dx, Dy, Dz;
extern sat_att_combined *pcombined_sat_att;
extern imu_filter *pfilt_att;
extern lsm9ds1_t *pBdata;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void CalAttitudeError(sat_att_combined *pcombined_sat_att) {

	//desired attitude
	float desired_roll = 0.0f;
	float desired_pitch = 0.0f;
	float desired_yaw = 0.0f;

	//current attitude
	float current_roll = pcombined_sat_att->roll;
	float current_pitch = pcombined_sat_att->pitch;
	float current_yaw = pcombined_sat_att->yaw;

	myprintf("Attitude Error Calculating\n");
	//attitude error calculation
	attitude_error[0] = desired_roll - current_roll;
	attitude_error[1] = desired_pitch - current_pitch;
	attitude_error[2] = desired_yaw - current_yaw;
	myprintf(" roll error = %.2f\r\n", attitude_error[0]);
	myprintf(" pitch error = %.2f\r\n", attitude_error[1]);
	myprintf(" yaw error = %.2f\r\n", attitude_error[2]);
}

void MTQ_Enable() {
	HAL_GPIO_WritePin(GPIOB, MTQEN_5V_Pin, SET);
}

void MTQ_Disable() {
	HAL_GPIO_WritePin(GPIOB, MTQEN_5V_Pin, RESET);
}

void takedecision(sat_att_combined *pcombined_sat_att, imu_filter *pfilt_att,
		lsm9ds1_t *pBdata) {
	CalAttitudeError(pcombined_sat_att);

	pBdata->mag.mx = mx;
	pBdata->mag.my = my;
	pBdata->mag.mz = mz;

	if (fabs(attitude_error[0]) > ERROR_THRESHOLD
			|| fabs(attitude_error[1]) > ERROR_THRESHOLD
			|| fabs(attitude_error[2]) > ERROR_THRESHOLD) {
		MTQ_Enable();
		CalTorque(pfilt_att, pBdata, pcombined_sat_att);
		SET_PWM_DUTY_CYCLE_Y(Dy);
		SET_PWM_DUTY_CYCLE_Z(Dz);
		myprintf("MTQ Active");

	}

	else {

		MTQ_Disable();
	}

}
//magnetic field from magnetometer
void CalTorque(imu_filter *pfilt_att, lsm9ds1_t *pBdata,
		sat_att_combined *pcombined_sat_att) {
	//angular velocities from RCfilter

	float Wx = -filt_imu.r_rps;
	float Wy = -filt_imu.q_rps;
	float Wz = filt_imu.p_rps;

	//magnetic fields from magnetometer
	float Bx = pBdata->mag.mz;
	float By = pBdata->mag.my;
	float Bz = -pBdata->mag.mx;

	myprintf(" wx = %.2f\r\n", Wx);
	myprintf(" wy = %.2f\r\n", Wy);
	myprintf(" wz = %.2f\r\n", Wz);

	myprintf(" Bx = %.2f\r\n", Bx);
	myprintf(" By = %.2f\r\n", By);
	myprintf(" Bz = %.2f\r\n", Bz);



	//calculate rate of change of magnetic field (dB/dt)
	float dBx_dt = Wx * Bz - Wz * By;
	float dBy_dt = Wz * Bx - Wx * Bz;
	float dBz_dt = Wx * By - Wy * Bx;

	myprintf("Desired Magnetic Moment\n");
	//calculate the desired magnetic moment
	float MomentX = -Kp * dBx_dt;
	float MomentY = -Kp * dBy_dt;
	float MomentZ = -Kp * dBz_dt;
	myprintf(" MomentX = %.2f\r\n", MomentX);
	myprintf(" MomentY = %.2f\r\n", MomentY);
	myprintf(" MomentZ = %.2f\r\n", MomentZ);
	myprintf(" MAX_MOMENT_MTQ = %.2f\r\n", MAX_MOMENT_MTQ);
	myprintf(" MAX_DUTY_CYCLE = %.2f\r\n", MAX_DUTY_CYCLE);
	//calculate torque
//	float tau_x = MomentY * Bz - MomentZ * By;
//	float tau_y = MomentZ * Bx - MomentX * Bz;
//	float tau_z = MomentX * By - MomentY * Bx;

//calculate PWM duty cycles based on maximum moment
	Dx = (MomentX / MAX_MOMENT_MTQ) * MAX_DUTY_CYCLE;
	Dy = (MomentY / MAX_MOMENT_MTQ) * MAX_DUTY_CYCLE;
	Dz = (MomentZ / MAX_MOMENT_MTQ) * MAX_DUTY_CYCLE;
	myprintf("Required Duty Cycle\n");
	myprintf(" Dx = %.2f\r\n", Dx);
	myprintf(" Dy = %.2f\r\n", Dy);
	myprintf(" Dz = %.2f\r\n", Dz);

	//Constrain the duty cycle
	if (Dx > MAX_DUTY_CYCLE)
		Dx = MAX_DUTY_CYCLE;
	if (Dy > MAX_DUTY_CYCLE)
		Dy = MAX_DUTY_CYCLE;
	if (Dz > MAX_DUTY_CYCLE)
		Dz = MAX_DUTY_CYCLE;
	if (Dx < 0.0f)
		Dx = 0.0f;
	if (Dy < 0.0f)
		Dy = 0.0f;
	if (Dz < 0.0f)
		Dz = 0.0f;
}

void SET_PWM_DUTY_CYCLE_Y(uint8_t Dy) {

	TIM3->CCR1 = Dy;
	TIM3->CCR2 = Dy;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}

void SET_PWM_DUTY_CYCLE_Z(uint8_t Dz) {

	TIM3->CCR3 = Dz;
	TIM3->CCR4 = Dz;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

}

//void SET_PWM_DUTY_CYCLE_Z(uint8_t Dz) {
//	TIM4->CCR3 = Dz;
//	TIM4->CCR4 = Dz;
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
//
//}
