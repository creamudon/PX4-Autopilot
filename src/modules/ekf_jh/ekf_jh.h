/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/front_distance_l.h>
#include <uORB/topics/front_distance_r.h>
#include <uORB/topics/dwm1000_distance.h>
#include <uORB/topics/ekf_jh.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/central_rotor.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>


#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>



#define DT 0.1f

using namespace time_literals;

extern "C" __EXPORT int ekf_jh_main(int argc, char *argv[]);


class EKF_JH : public ModuleBase<EKF_JH>, public ModuleParams
{
public:
	EKF_JH(int example_param, bool example_flag);

	virtual ~EKF_JH() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static EKF_JH *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:



	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);




	//============ MATH FUNCTION ====================

	//===============================================

	//============   EKF ============================

	void Hjacob(float xp[4],float H[5][4],float uwb_1[3], float uwb_2[3]){
		H[0][0]=1;
		H[0][1]=0;
		H[0][2]=0;
		H[0][3]=0;

		float norm1=sqrtf( powf(xp[0]-uwb_1[0],2) + powf(xp[1]-uwb_1[1],2) + powf(xp[2]-uwb_1[2],2) );
		float norm2=sqrtf( powf(xp[0]-uwb_2[0],2) + powf(xp[1]-uwb_2[1],2) + powf(xp[2]-uwb_2[2],2) );

		H[1][0]=(xp[0]-uwb_1[0])/norm1;
		H[1][1]=(xp[1]-uwb_1[1])/norm1;
		H[1][2]=(xp[2]-uwb_1[2])/norm1;
		H[1][3]=0;

		H[2][0]=(xp[0]-uwb_2[0])/norm2;
		H[2][1]=(xp[1]-uwb_2[1])/norm2;
		H[2][2]=(xp[2]-uwb_2[2])/norm2;
		H[2][3]=0;

		H[3][0]=0;
		H[3][1]=0;
		H[3][2]=1;
		H[3][3]=0;

		H[4][0]=0;
		H[4][1]=0;
		H[4][2]=0;
		H[4][3]=1;



	}
	void hx(float xhat[4], float zp[5],float uwb_1[3], float uwb_2[3]){


		float dis_1=sqrtf( powf(xhat[0]-uwb_1[0],2) + powf(xhat[1]-uwb_1[1],2) + powf(xhat[2]-uwb_1[2],2) );
		float dis_2=sqrtf( powf(xhat[0]-uwb_2[0],2) + powf(xhat[1]-uwb_2[1],2) + powf(xhat[2]-uwb_2[2],2) );

		zp[0]=xhat[0];
		zp[1]=dis_1;
		zp[2]=dis_2;
		zp[3]=xhat[2];
		zp[4]=xhat[3];


	}

	//===============================================

	param_t uwb_1_x_param_handle = PARAM_INVALID;
	param_t uwb_1_y_param_handle = PARAM_INVALID;
	param_t uwb_1_z_param_handle = PARAM_INVALID;

	param_t uwb_2_x_param_handle = PARAM_INVALID;
	param_t uwb_2_y_param_handle = PARAM_INVALID;
	param_t uwb_2_z_param_handle = PARAM_INVALID;



	// float lidar_angle = 45 * 3.141592f / 180.0f;//rad
	float lidar_angle = 60.0f * 3.141592f / 180.0f;//rad
	// float lidars_distance = 0.21;
	float lidars_distance = 0.118f;

	float Calc_yaw_with_ToF(float d1,float d2);
	float Calc_front_distance(float d1, float d2, float theta);
	void Q_to_RPY();
	float PWM_to_N(int pwm);

	float Quad_T=0.0f;

	uint64_t delta_t=0;
	uint64_t prev_t=0;

	float delta_t_s=0.0f;

	double X=0.0f;
	double Y=0.0f;
	double Z=0.0f;
	double V_y=0.0f;


	void Calc_Quad_force(float global_N[],float N,float roll_, float pitch_, float yaw_);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};


	// uORB::Subscription _vehicle_odometry_sub {ORB_ID(vehicle_odometry)};
	// struct vehicle_odometry_s odom_s;

	uORB::Subscription _front_distance_l_sub {ORB_ID(front_distance_l)};
	uORB::Subscription _front_distance_r_sub {ORB_ID(front_distance_r)};
	uORB::Subscription _dwm1000_distance_sub {ORB_ID(dwm1000_distance)};
	uORB::Subscription _distance_sensor_sub {ORB_ID(distance_sensor)};

	uORB::Subscription _vehicle_attitude_sub {ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_odometry_sub {ORB_ID(vehicle_odometry)};

	uORB::Subscription _central_rotor_sub {ORB_ID(central_rotor)};


	// uORB::Subscription _actuators_0_sub {ORB_ID(actuator_controls_0)};
	uORB::Subscription _actuator_outputs_sub {ORB_ID(actuator_outputs),2};




	struct front_distance_l_s front_dist_l_s;
	struct front_distance_r_s front_dist_r_s;
	struct dwm1000_distance_s uwb_distance_s;
	struct distance_sensor_s dist_sensor_s;

	struct vehicle_attitude_s att_s;
	struct vehicle_odometry_s odom_s;

	struct central_rotor_s cent_rotor_s;
	struct actuator_outputs_s actuator_out_s;


	uORB::PublicationMulti<ekf_jh_s> _ekf_jh_pub{ORB_ID(ekf_jh)};

	ekf_jh_s report{};



	orb_advert_t mavlink_log_pub_user=NULL;
	char *gnc_test_str;
	int count_d=0;


	float roll=0.0f;
	float pitch=0.0f;
	float yaw=0.0f;

	float odom_vx=0.0f;
	float odom_vy=0.0f;
	float odom_vz=0.0f;

};

