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

#include "ekf_jh.h"

#include "MedianFilter.h"
#include "uwb_ekf.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#define r2d				((float)(57.295779513082323)) // radian to degree unit
#define d2r				1.0f/((float)(57.295779513082323)) // radian to degree unit


// void EKF_JH::Median_filter(){

// }


#define MEDIAN_WINDOW 15
static sMedianFilter_t down_filter;
static sMedianNode_t down_filter_Buffer[MEDIAN_WINDOW];

static sMedianFilter_t uwb_1_filter;
static sMedianNode_t uwb_1_filter_Buffer[MEDIAN_WINDOW];

static sMedianFilter_t uwb_2_filter;
static sMedianNode_t uwb_2_filter_Buffer[MEDIAN_WINDOW];



void EKF_JH::Q_to_RPY(){



		roll   = atan2f(    2*(att_s.q[0]*att_s.q[1] + att_s.q[2]*att_s.q[3]),
			        1 - 2*(att_s.q[1]*att_s.q[1] + att_s.q[2]*att_s.q[2]))*r2d;
		pitch  =  asinf(    2*(att_s.q[0]*att_s.q[2] - att_s.q[3]*att_s.q[1]))*r2d;
		yaw    = atan2f(    2*(att_s.q[0]*att_s.q[3] + att_s.q[1]*att_s.q[2]),
				1 - 2*(att_s.q[2]*att_s.q[2] + att_s.q[3]*att_s.q[3]))*r2d;

}


float EKF_JH::PWM_to_N(int pwm){

	float N=0;
	if(pwm>1400){
		N=0.0268f*pwm-34.409f;
	}
	else if(pwm>1100){
		N=0.00004f*pwm*pwm-0.0953f*N+53.281f;
	}
	else{
		N=0;
	}

	return N;

}

void EKF_JH::Calc_Quad_force(float global_N[],float N,float roll_, float pitch_, float yaw_){

	global_N[0]=N*(float)sin(pitch_*d2r);
	global_N[1]=-N*(float)cos(pitch_*d2r)*(float)sin(roll_*d2r);
	global_N[2]=-N*(float)cos(roll_*d2r)*(float)cos(pitch_*d2r)+ 50.995f; // mg

}


/*d1 = left ToF sensor's measurement [m], d2 = left ToF sensor's measurement [m]*/
float EKF_JH::Calc_yaw_with_ToF(float d1,float d2){

	float theta=(float)atan( ((d1-d2)*(float)sin(lidar_angle)) / (lidars_distance-(d1+d2)*(float)cos(lidar_angle)) ) ;
	// radian? degre????

	return theta;




}

/*d1 = left ToF sensor's measurement [m], d2 = left ToF sensor's measurement [m], theta = calculated yaw [rad]*/
float EKF_JH::Calc_front_distance(float d1, float d2, float theta){

	float d_front=  ( d1* (float)sin(lidar_angle-theta)  +  d2 *(float)sin(lidar_angle-theta) ) / 2.0f;

	return d_front;

}


int EKF_JH::print_status()
{
	PX4_INFO("Running EKF_JH");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int EKF_JH::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int EKF_JH::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

EKF_JH *EKF_JH::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	EKF_JH *instance = new EKF_JH(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

EKF_JH::EKF_JH(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void EKF_JH::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;


	uwb_1_x_param_handle  = param_find("UWB_ANCHOR_1_X");
	uwb_1_y_param_handle  = param_find("UWB_ANCHOR_1_Y");
	uwb_1_z_param_handle  = param_find("UWB_ANCHOR_1_Z");

	uwb_2_x_param_handle  = param_find("UWB_ANCHOR_2_X");
	uwb_2_y_param_handle  = param_find("UWB_ANCHOR_2_Y");
	uwb_2_z_param_handle  = param_find("UWB_ANCHOR_2_Z");



	// initialize parameters
	parameters_update(true);




	// Median Filter
	down_filter.numNodes = MEDIAN_WINDOW;
   	down_filter.medianBuffer = down_filter_Buffer;

	uwb_1_filter.numNodes=MEDIAN_WINDOW;
	uwb_1_filter.medianBuffer=uwb_1_filter_Buffer;

	uwb_2_filter.numNodes=MEDIAN_WINDOW;
	uwb_2_filter.medianBuffer=uwb_2_filter_Buffer;


        MEDIANFILTER_Init(&down_filter);
	MEDIANFILTER_Init(&uwb_1_filter);
	MEDIANFILTER_Init(&uwb_2_filter);

	uint64_t timestamp_last_run = 0;





	while (!should_exit()) {



		// Get current timestamp
        	uint64_t timestamp_now = hrt_absolute_time();

       		// // Calculate time passed since last run
        	uint64_t time_since_last_run = timestamp_now - timestamp_last_run;

       		// Check if 1000ms have passed since last run
		if (time_since_last_run >= 1e5) {
          	  // Do something here




		delta_t=hrt_absolute_time()- prev_t;
		delta_t_s=(float)delta_t/1000000.0f;
		prev_t=hrt_absolute_time();

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			// px4_usleep(1000000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...


			_dwm1000_distance_sub.update(&uwb_distance_s);
			_front_distance_l_sub.update(&front_dist_l_s);
			_front_distance_r_sub.update(&front_dist_r_s);
			_distance_sensor_sub.update(&dist_sensor_s);

			if(_central_rotor_sub.update(&cent_rotor_s)){
				for(int i=0;i<3;i++){
				   report.central_force[i]=cent_rotor_s.central_force[i];
				}

			}

			if(_actuator_outputs_sub.update(&actuator_out_s)){

				float Quad_T_temp=0.0f;
				for(int i=0;i<4;i++){
					report.actuators[i]=actuator_out_s.output[i];

					Quad_T_temp+=PWM_to_N(report.actuators[i]);
				}

				Quad_T=Quad_T_temp;

				float global_N[3];
				Calc_Quad_force(global_N,Quad_T,roll, pitch, yaw);

				for(int i=0;i<3;i++){
					report.quad_force[i]=global_N[i];
				}





			}


			if(_vehicle_attitude_sub.update(&att_s)){
				Q_to_RPY();

				report.roll=roll;
				report.pitch=pitch;
				report.yaw=yaw;

			}


			if(_vehicle_odometry_sub.update(&odom_s)){

			switch (odom_s.velocity_frame) {
			case vehicle_odometry_s::BODY_FRAME_FRD:
				odom_vx = odom_s.vx;
				odom_vy = odom_s.vy;
				odom_vz = odom_s.vz;
				break;

			case vehicle_odometry_s::LOCAL_FRAME_FRD:
			case vehicle_odometry_s::LOCAL_FRAME_NED:
			// Body frame to local frame
			const matrix::Dcmf R_body_to_local(matrix::Quatf(odom_s.q));

			// Rotate linear velocity from local to body frame
			const matrix::Vector3f linvel_body(R_body_to_local.transpose() *
								   matrix::Vector3f(odom_s.vx, odom_s.vy, odom_s.vz));

			odom_vx = linvel_body(0);
			odom_vy = linvel_body(1);
			odom_vz = linvel_body(2);
			break;
			}

			report.odom_vx=odom_vx;
			report.odom_vy=odom_vy;
			report.odom_vz=odom_vz;


			}







			float distance_down=dist_sensor_s.current_distance * (float)cos(pitch * d2r) * (float)cos(pitch * d2r) ;


			float distance_down_filtered = MEDIANFILTER_Insert(&down_filter, distance_down);


			float yaw_wall=Calc_yaw_with_ToF(front_dist_l_s.distance_m_l,front_dist_r_s.distance_m_r); //rad

			float distance_front=Calc_front_distance(front_dist_l_s.distance_m_l,front_dist_r_s.distance_m_r,yaw_wall);






			report.timestamp = hrt_absolute_time();
			report.distance_down=distance_down;



			float distance_uwb_1=uwb_distance_s.anchor_distance[0]/1000.0f;
			float distance_uwb_2=uwb_distance_s.anchor_distance[1]/1000.0f;

			float distance_uwb_1_filtered = MEDIANFILTER_Insert(&uwb_1_filter, distance_uwb_1);
			float distance_uwb_2_filtered = MEDIANFILTER_Insert(&uwb_2_filter, distance_uwb_2);


			report.distance_uwb_1=distance_uwb_1;
			report.distance_uwb_2=distance_uwb_2;


			report.distance_uwb_1_filtered=distance_uwb_1_filtered;
			report.distance_uwb_2_filtered=distance_uwb_2_filtered;

			// double z[5]={(double)((distance_front+0.32f)*-1.0f),(double)(distance_uwb_1_filtered+0.2f +0.1f),(double)(distance_uwb_2_filtered+0.2f -0.1f),(double)((distance_down_filtered-0.84f)*-1.0f),(double)odom_vx};
			// double u[3]={(double)((report.quad_force[0]+report.central_force[0])/5.2f) , (double)((report.quad_force[0]+report.central_force[1])/5.2f), (double)((report.quad_force[0]+report.central_force[2])/5.2f)};

			// UWBEKF_vel_2UWB_with_Input(z,u,(double)delta_t_s,&X,&Y,&Z,&V_y);

			report.x=(float)X;
			report.y=(float)Y;
			report.z=(float)Z;



			report.yaw_wall=yaw_wall * 180.0f / 3.141592f;;
			report.distance_front=distance_front;

			report.distance_down_filtered=distance_down_filtered;

			report.delta_t_s=delta_t_s;




			_ekf_jh_pub.publish(report);


			// if(_vehicle_odometry_sub.update(&odom_s)){

			// }




		}

		parameters_update();

		 // Update last run timestamp
	        timestamp_last_run = timestamp_now;

		}

		uint64_t time_until_next_run = 1e5 - time_since_last_run;
        	usleep(time_until_next_run);

	}

	orb_unsubscribe(sensor_combined_sub);
}

void EKF_JH::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();

		float uwb_1_pos[3] = {0,0,0};
		float uwb_2_pos[3] = {0,0,0};




		param_get(uwb_1_x_param_handle, &uwb_1_pos[0]);
		param_get(uwb_1_y_param_handle, &uwb_1_pos[1]);
		param_get(uwb_1_z_param_handle, &uwb_1_pos[2]);

		param_get(uwb_2_x_param_handle, &uwb_2_pos[0]);
		param_get(uwb_2_y_param_handle, &uwb_2_pos[1]);
		param_get(uwb_2_z_param_handle, &uwb_2_pos[2]);


		for(int i=0;i<3;i++){
			report.uwb_anchor_1_pos[i]=uwb_1_pos[i];
			report.uwb_anchor_2_pos[i]=uwb_2_pos[i];
		}






	}
}

int EKF_JH::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ekf_jh_main(int argc, char *argv[])
{
	return EKF_JH::main(argc, argv);
}
