/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

#define R2D				((float)(57.295779513082323)) // radian to degree unit
#define D2R				1.0f/((float)(57.295779513082323)) // radian to degree unit

PID_t _Central_rotor_vx_ctrl{};
PID_t _Central_rotor_vy_ctrl{};

PID_t _Central_rotor_x_ctrl{};
PID_t _Central_rotor_y_ctrl{};

float angle1=0;
float angle2=0;

float vx=0;
float vy=0;
float vz=0;

float global_x=0;
float global_y=0;
float global_z=0;

float EDF_Throttle=0;
float EDF_Force=0;


float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Throttle_to_EDF_Force(float th_in){

	return 21.832f*th_in*th_in-0.0932f*th_in-0.0058f;

}

void
MulticopterRateControl:: Calc_InverseKinematics(float * theta1, float * theta2, float F_x, float F_y, float F_central){


	//Normalization

	float norm=sqrt(F_x*F_x+F_y*F_y);
	if(norm>F_central){
		F_x=F_x/norm*F_central;
		F_y=F_y/norm*F_central;
	}


	float den =   F_central*F_central - F_x*F_x - F_y*F_y ;
	if(den<0) den = 0;



	//InverseKinematics
	float angle1_temp=(float)atan(F_x / (float)-sqrt(den) ) * 180.f/3.141592f;

	*theta1= (PX4_ISFINITE(angle1_temp)) ? angle1_temp : 0.0f;

	//old .. wrong I.K
	// float angle2_temp = (float)asin(-F_y/F_central) * 180.f/3.141592f;

	//New ! Correct I.K
	float angle2_temp = (float)asin(F_y/F_central) * 180.f/3.141592f;

	*theta2= (PX4_ISFINITE(angle2_temp)) ? angle2_temp : 0.0f;

	// count_s++;
	// if(count_s>100){
	// 	count_s=0;

	// 	double temp=-sqrt( F_central*F_central - F_x*F_x - F_y*F_y );
	// 	// mavlink_log_info(&mavlink_log_pub_user,"%.3f %.3f %.3f %.3f",(double)F_x,(double)F_y,temp ,(double)angle1_temp );

	// 	mavlink_log_info(&mavlink_log_pub_user,"%.3f %.3f %.3f %.3f %.3f %.3f",(double)pow(F_x,2),(double)pow(F_y,2), (double) F_x, (double)F_y, (double)pow(F_central,2),temp);

	// }


}

// Need to check
void Calc_Forward_Kinematics(float theta1,float theta2,float F_central,float rotor_Force[]){

	float Force_rotor_x=-F_central*sin(theta1*D2R)*cos(theta2*D2R);
	float Force_rotor_y=F_central*sin(theta2*D2R);
	float Force_rotor_z=-F_central*cos(theta1*D2R)*cos(theta2*D2R);

	rotor_Force[0]=Force_rotor_x;
	rotor_Force[1]=Force_rotor_y;
	rotor_Force[2]=Force_rotor_z;

	// float Force_rotor[3]={Force_rotor_x,Force_rotor_y,Force_rotor_z};
}

float MN4006_Inverse_Mapping(float Force){

	//Inverse of f(x)=115.66*x^2-11.834*x-1.2261
	float Mapping = 0.004323f*sqrt(462.64f*Force+707.29f)+0.051159f;

	float control = (PX4_ISFINITE(Mapping)) ? Mapping : 0.0f;

	return control;
}



void Central_rotor_PID_parameters_update(){

	// pid_init(&_Central_rotor_vx_ctrl,PID_MODE_DERIVATIV_CALC,0.001);
	// pid_set_parameters(&_Central_rotor_vx_ctrl,2,0.5,0.01,1.0,2.0f);

	// pid_init(&_Central_rotor_vy_ctrl,PID_MODE_DERIVATIV_CALC,0.001);
	// pid_set_parameters(&_Central_rotor_vy_ctrl,2,0.5,0.01,1.0,2.0f);

	pid_init(&_Central_rotor_vx_ctrl,PID_MODE_DERIVATIV_CALC,0.001);
	// pid_set_parameters(&_Central_rotor_vx_ctrl,2,0.5,0.01,1.0,EDF_Throttle*0.8f);
	pid_set_parameters(&_Central_rotor_vx_ctrl,2,0.5,0.01,1.0,EDF_Force); //Throttle?!?!?!??!?!?!? it should be a EDF

	pid_init(&_Central_rotor_vy_ctrl,PID_MODE_DERIVATIV_CALC,0.001);
	// pid_set_parameters(&_Central_rotor_vy_ctrl,2,0.5,0.01,1.0,EDF_Throttle*0.8f);
	pid_set_parameters(&_Central_rotor_vy_ctrl,2,0.5,0.01,1.0,EDF_Force);


	//================ Position PID =====================

	pid_init(&_Central_rotor_x_ctrl,PID_MODE_DERIVATIV_CALC,0.001);
	pid_set_parameters(&_Central_rotor_x_ctrl,0.5,0,0,1.0,1);

	pid_init(&_Central_rotor_y_ctrl,PID_MODE_DERIVATIV_CALC,0.001);
	pid_set_parameters(&_Central_rotor_y_ctrl,0.5,0,0,1.0,1);



}

void
MulticopterRateControl::Central_rotor_Control_Position_PID(float dt, float sp_x, float sp_y){




	/* PID Control*/

		switch (odom_s.velocity_frame) {
		case vehicle_odometry_s::BODY_FRAME_FRD:
			vx = odom_s.vx;
			vy = odom_s.vy;
			vz = odom_s.vz;
			break;

		case vehicle_odometry_s::LOCAL_FRAME_FRD:
		case vehicle_odometry_s::LOCAL_FRAME_NED:
			// Body frame to local frame
			const matrix::Dcmf R_body_to_local(matrix::Quatf(odom_s.q));

			// Rotate linear velocity from local to body frame
			const matrix::Vector3f linvel_body(R_body_to_local.transpose() *
								   matrix::Vector3f(odom_s.vx, odom_s.vy, odom_s.vz));

			vx = linvel_body(0);
			vy = linvel_body(1);
			vz = linvel_body(2);
			break;
		}


	//X - Axis
	float x_control=pid_calculate(&_Central_rotor_x_ctrl,sp_x,x,vx,dt);
	float vx_control=pid_calculate(&_Central_rotor_vx_ctrl,x_control,vx,pos_s.ax,dt); // Pos_s.ax???????????????

	// Y - Axis
	float y_control=pid_calculate(&_Central_rotor_y_ctrl,sp_y,y,vx,dt);
	float vy_control=pid_calculate(&_Central_rotor_vy_ctrl,y_control,vy,pos_s.ay,dt); // Pos_s.ay????????????????

	// if(y_control_enable==true){

	// }
	// else{

	// }




	Calc_InverseKinematics(&angle1,&angle2,vx_control,vy_control,EDF_Force);


	/* Debug */

	// count_d++;
	// if(count_d>100){
	// 	count_d=0;
	// 	// PX4_INFO("%.3f %.3f %.3f",(double)vx_control,(double)pos_s.vx,(double)angle1);
	// 	mavlink_log_info(&mavlink_log_pub_user,"PID : %.3f %.3f %.3f %.3f",(double)vx_sp,(double)vy_sp,(double)angle1,(double)angle2);
	// }


}

void
MulticopterRateControl::Central_rotor_Control_PID(float dt, float sp_x, float sp_y){


	/* Input */
	float vx_sp=map(sp_x,-1,1,-2,2);
	float vy_sp=map(sp_y,-1,1,-2,2);

	// float vx_sp=map(sp_x,-1,1,-1,1);
	// float vy_sp=map(sp_y,-1,1,-1,1);

	/* Manual Control (InverseKinematics)*/

	// Calc_InverseKinematics(&angle1,&angle2,vx_sp,vy_sp,3.0);

	/* Manual Control (real Manual)*/

	//  angle1=map(vx_sp,-10,10,-90,90);
	//  angle2=map(vy_sp,-10,10,-90,90);

	/* PID Control*/

		switch (odom_s.velocity_frame) {
		case vehicle_odometry_s::BODY_FRAME_FRD:
			vx = odom_s.vx;
			vy = odom_s.vy;
			vz = odom_s.vz;
			break;

		case vehicle_odometry_s::LOCAL_FRAME_FRD:
		case vehicle_odometry_s::LOCAL_FRAME_NED:
			// Body frame to local frame
			const matrix::Dcmf R_body_to_local(matrix::Quatf(odom_s.q));

			// Rotate linear velocity from local to body frame
			const matrix::Vector3f linvel_body(R_body_to_local.transpose() *
								   matrix::Vector3f(odom_s.vx, odom_s.vy, odom_s.vz));

			vx = linvel_body(0);
			vy = linvel_body(1);
			vz = linvel_body(2);
			break;

		}

	//============= Velocity PID ==============================
	// // // PID
	float vx_control=pid_calculate(&_Central_rotor_vx_ctrl,vx_sp,vx,pos_s.ax,dt); // Pos_s.ax???????????????
	float vy_control=pid_calculate(&_Central_rotor_vy_ctrl,vy_sp,vy,pos_s.ay,dt); // Pos_s.ay????????????????

	// Calc_InverseKinematics(&angle1,&angle2,vx_control,vy_control,EDF_Force);
	Calc_InverseKinematics(&angle1,&angle2,vx_control,vy_control,3.0f);
	//============================================

	/* Debug */

	// count_d++;
	// if(count_d>100){
	// 	count_d=0;
	// 	// PX4_INFO("%.3f %.3f %.3f",(double)vx_control,(double)pos_s.vx,(double)angle1);
	// 	mavlink_log_info(&mavlink_log_pub_user,"PID : %.3f %.3f %.3f %.3f",(double)vx_sp,(double)vy_sp,(double)angle1,(double)angle2);
	// }


}



MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	//========== Edited by JH ===============
	Central_rotor_PID_parameters_update();
	//=====================================

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		if (_landing_gear_sub.updated()) {
			landing_gear_s landing_gear;

			if (_landing_gear_sub.copy(&landing_gear)) {
				if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
					if (landing_gear.landing_gear == landing_gear_s::GEAR_UP && (_landed || _maybe_landed)) {
						mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear\t");
						events::send(events::ID("mc_rate_control_not_retract_landing_gear_landed"),
						{events::Log::Error, events::LogInternal::Info},
						"Landed, unable to retract landing gear");

					} else {
						_landing_gear = landing_gear.landing_gear;
					}
				}
			}
		}

		//=============== Edited by JH =======================
		// /* get switches */ //added by JH


		// _manual_control_switches_sub.update(&manual_control_switches);

		central_rotor_s central_rotor_report{};



		//============ Edied by JH ================
		_ekf_jh_sub.update(&ekf_wall_s);

		if(_input_rc_sub.update(&rc_s)){
			if(rc_s.values[6]>1300) central_rotor_contol_enable=true; //Right Slide
			// if(rc_s.values[5]>1800) central_rotor_contol_enable=true;
			else	central_rotor_contol_enable=false;

			if(rc_s.values[5]>1200){
				wall_attaching_mode=true;
			}
			else {
				wall_attaching_mode=false;
			}

			if(wall_attaching_mode==true){
				x_setpoint =0.0f;

				if(rc_s.values[5]>1500){

					y_control_enable = true;

				}

				if(rc_s.values[5]>1900){

					y_setpoint =2.0f;

				}

				else if(rc_s.values[5]>1500){
					y_setpoint =0.0f;

				}


			}






			EDF_Throttle=map(rc_s.values[6],1000,2000,0,1);
			EDF_Throttle= (EDF_Throttle > 0.8f) ? 0.8f : EDF_Throttle;
			EDF_Force= Throttle_to_EDF_Force(EDF_Throttle);

		}





		_vehicle_local_position_sub.update(&pos_s);
		if(_vehicle_odometry_sub.update(&odom_s)){
			//vx convert

			//positiom convert
		}

		//==============================================
		//================= Edited by JH ============================

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint_rotor)) {



				// // manual rates control - ACRO mode
				// const Vector3f man_rate_sp{
				// 	math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
				// 	math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
				// 	math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				// _rates_sp = man_rate_sp.emult(_acro_rate_max);
				// _thrust_sp = math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);

				// // publish rate setpoint
				// vehicle_rates_setpoint_s v_rates_sp{};
				// v_rates_sp.roll = _rates_sp(0);
				// v_rates_sp.pitch = _rates_sp(1);
				// v_rates_sp.yaw = _rates_sp(2);
				// v_rates_sp.thrust_body[0] = 0.0f;
				// v_rates_sp.thrust_body[1] = 0.0f;
				// v_rates_sp.thrust_body[2] = -_thrust_sp;
				// v_rates_sp.timestamp = hrt_absolute_time();

				// _v_rates_sp_pub.publish(v_rates_sp);
		}

		//===========================================================
		// count_d++;
		// if(count_d>100){
		// count_d=0;

		// // mavlink_log_info(&mavlink_log_pub_user,"ratecontrol : %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",
		// // (double)odom_s.local_frame,(double)odom_s.velocity_frame,(double)odom_s.vx,(double)odom_s.vy,(double)odom_s.vz,(double)odom_s.x,(double)odom_s.y,(double)odom_s.z,(double)manual_control_setpoint_rotor.x,(double)manual_control_setpoint_rotor.y);

		// // mavlink_log_info(&mavlink_log_pub_user,"ratecontrol : %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",
		// // (double)odom_s.local_frame,(double)odom_s.velocity_frame,(double)odom_s.vx,(double)vx,(double)odom_s.vy,(double)vy);

		// mavlink_log_info(&mavlink_log_pub_user,"ratecontrol : %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f",
		// (double)EDF_Throttle,(double)EDF_Force,(double)odom_s.vx,(double)vx,(double)odom_s.vy,(double)vy);

		// }

		Central_rotor_Control_PID(dt, manual_control_setpoint_rotor.x, manual_control_setpoint_rotor.y);

		float central_force[3];

		Calc_Forward_Kinematics(angle1,angle2,EDF_Force,central_force);
		// float Central_Rotor_Force_z=Calc_Forward_Kinematics(angle1,angle2,EDF_Force);
		// float z_compensation=MN4006_Inverse_Mapping(Central_Rotor_Force_z);



		central_rotor_report.timestamp=hrt_absolute_time();
		central_rotor_report.edf_force=EDF_Force;
		central_rotor_report.edf_throttle=EDF_Throttle;
		central_rotor_report.angle1=angle1;
		central_rotor_report.angle2=angle2;
		for(int i=0;i<3;i++){
			central_rotor_report.central_force[i]=central_force[i];
		}
		central_rotor_report.mode=(uint8_t)central_rotor_contol_enable;
		central_rotor_report.attaching_mode=(uint8_t)wall_attaching_mode;

		central_rotor_report.x_setpoint=x_setpoint;
		central_rotor_report.y_setpoint=y_setpoint;

		_central_rotor_pub.publish(central_rotor_report);


		if (_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
				_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
				_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;

			// ================ Edited By JH =================
			if(central_rotor_contol_enable==true){
				float _angle1_sp=map(angle1,-90,90,-1,1);

				//Old .. servo2 direction is CW
				// float _angle2_sp=map(angle2,-90,90,-1,1);

				//New ! CCW Direction
				float _angle2_sp=map(angle2,-90,90,1,-1);

				actuators.control[4] = PX4_ISFINITE(_angle1_sp) ? _angle1_sp : 0.0f;
				actuators.control[5] = PX4_ISFINITE(_angle2_sp) ? _angle2_sp : 0.0f;
				actuators.control[6] = PX4_ISFINITE(EDF_Throttle) ? EDF_Throttle: 0.0f;

			}
			else{
				actuators.control[4] = 0.0f;
				actuators.control[5] = 0.0f;
				actuators.control[6] = 0.0f;
			}
			// count_d++;
			// if(count_d>100){
			// count_d=0;
			// // PX4_INFO("%.3f %.3f %.3f",(double)vx_control,(double)pos_s.vx,(double)angle1);
			// mavlink_log_info(&mavlink_log_pub_user,"%.3f %.3f %.3f %f",(double)actuators.control[4],(double)actuators.control[5],(double)actuators.control[6],(double)manual_control_switches.gear_switch);
			// }


			// actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			//==============================================
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			if (!_vehicle_status.is_vtol) {
				publishTorqueSetpoint(att_control, angular_velocity.timestamp_sample);
				publishThrustSetpoint(angular_velocity.timestamp_sample);
			}

			// scale effort by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

			updateActuatorControlsStatus(actuators, dt);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void MulticopterRateControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = PX4_ISFINITE(_thrust_sp) ? -_thrust_sp : 0.0f; // Z is Down

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void MulticopterRateControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
