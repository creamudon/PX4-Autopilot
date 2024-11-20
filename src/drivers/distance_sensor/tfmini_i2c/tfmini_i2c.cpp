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

#include "tfmini_i2c.hpp"

//original file is VL53L0X

/* TFMINI_I2C Registers addresses */



#define VL53L0X_US                                      1000    // 1ms
// #define VL53L0X_SAMPLE_RATE                             50000   // 50ms
// #define TFMINI_I2C_SAMPLE_RATE                             100000  // 100ms
#define TFMINI_I2C_SAMPLE_RATE                             10000  // 10ms

#define VL53L0X_BUS_CLOCK                               400000 // 400kHz bus clock speed

TFMINI_I2C::TFMINI_I2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{

	// TFMINI typical range 0-2 meters with 25 degree field of view
	_px4_rangefinder.set_min_distance(0.1f);
	_px4_rangefinder.set_max_distance(12.f);
	_px4_rangefinder.set_fov(math::radians(2.f));

	// Allow retries as the device typically misses the first measure attempts.
	I2C::_retries = 10;

	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TFMINI);
}

TFMINI_I2C::~TFMINI_I2C()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFMINI_I2C::init()
{

	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}


	// mavlink_log_info(&mavlink_log_pub_user,"TFMINI_I2C_Init: %.3f",(double)ret);

	ScheduleNow();
	return PX4_OK;
}

int TFMINI_I2C::collect()
{

	// Read from the sensor.
	uint8_t val[7] {};
	perf_begin(_sample_perf);

	_collect_phase = false;

	// const hrt_abstime timestamp_sample = hrt_absolute_time();


	if (transfer(nullptr, 0, &val[0], 7) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	perf_end(_sample_perf);






	if(get_device_address()==0x10){

	front_distance_l_s report{};

	uint16_t distance_cm = val[2]| (val[3] << 8);
	float distance_m = distance_cm / 100.f;

	uint16_t Strength = val[4] | (val[5] << 8) ;


	report.distance_cm_l=distance_cm;
	report.distance_m_l=distance_m;
	report.strength_l=Strength;

	_front_distance_l_pub.publish(report);

	}

	else if(get_device_address()==0x11){

	front_distance_r_s report{};

	uint16_t distance_cm = val[2]| (val[3] << 8);
	float distance_m = distance_cm / 100.f;

	uint16_t Strength = val[4] | (val[5] << 8) ;

	report.timestamp = hrt_absolute_time();
	report.distance_cm_r=distance_cm;
	report.distance_m_r=distance_m;
	report.strength_r=Strength;

	_front_distance_r_pub.publish(report);

	}





	// count_d++;
	// if(count_d>100){
	// count_d=0;

	// 	mavlink_log_info(&mavlink_log_pub_user,"TFMINI_I2C_Collect: %.3f %.3f",(double)distance_m,(double)Strength);

	// }

	// mavlink_log_info(&mavlink_log_pub_user,"TFMINI_I2C_Collect: %.3f %.3f %.3f %.3f",(double)get_device_address(),(double)get_device_bus(),(double)distance_cm,(double)Strength);

	// _px4_rangefinder.update(timestamp_sample, distance_cm);// ..?

	return PX4_OK;
}

int TFMINI_I2C::measure()
{


/*

Send the command ‘0’ – read altitude
*/
//uint8_t cmd = 0;

// int ret = OK;//transfer(&cmd, 1, nullptr, 0);

uint8_t cmd[3] = {0x1,0x2,0x7};
int ret = transfer(&cmd[0], 3, nullptr, 0);


// mavlink_log_info(&mavlink_log_pub_user,"measure ret: %d cmd : %d %d %d ",ret,cmd[0],cmd[1],cmd[2]);

if (OK != ret) {
perf_count(_comms_errors);
PX4_DEBUG(“i2c::transfer returned %d”, ret);
return ret;
}

_collect_phase =true;

ret = OK;

return ret;
}


// int TFMINI_I2C::measure()
// {
// 	uint8_t wait_for_measurement = 0;
// 	uint8_t system_start = 0;

// 	// Send the command to begin a measurement.
// 	const uint8_t cmd = RESULT_RANGE_STATUS_REG + 10;

// 	if (_new_measurement) {

// 		_new_measurement = false;

// 		writeRegister(0x80, 0x01);
// 		writeRegister(0xFF, 0x01);
// 		writeRegister(0x00, 0x00);
// 		writeRegister(0x91, _stop_variable);
// 		writeRegister(0x00, 0x01);
// 		writeRegister(0xFF, 0x00);
// 		writeRegister(0x80, 0x00);

// 		writeRegister(SYSRANGE_START_REG, 0x01);

// 		readRegister(SYSRANGE_START_REG, system_start);

// 		if ((system_start & 0x01) == 1) {
// 			ScheduleDelayed(VL53L0X_US);
// 			return PX4_OK;

// 		} else {
// 			_measurement_started = true;
// 		}
// 	}

// 	if (!_collect_phase && !_measurement_started) {

// 		readRegister(SYSRANGE_START_REG, system_start);

// 		if ((system_start & 0x01) == 1) {
// 			ScheduleDelayed(VL53L0X_US);
// 			return PX4_OK;

// 		} else {
// 			_measurement_started = true;
// 		}
// 	}

// 	readRegister(RESULT_INTERRUPT_STATUS_REG, wait_for_measurement);

// 	if ((wait_for_measurement & 0x07) == 0) {
// 		ScheduleDelayed(VL53L0X_US); // reschedule every 1 ms until measurement is ready
// 		return PX4_OK;
// 	}

// 	_collect_phase = true;

// 	int ret = transfer(&cmd, sizeof(cmd), nullptr, 0);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		DEVICE_LOG("i2c::transfer returned %d", ret);
// 		return ret;
// 	}

// 	return PX4_OK;
// }

void TFMINI_I2C::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

int TFMINI_I2C::probe()
{
	// mavlink_log_info(&mavlink_log_pub_user,"TFMINI_I2C_probe:");
	if (sensorInit() == PX4_OK) {
		return PX4_OK;
	}

	// Device not found on any address.
	return -EIO;
}

int TFMINI_I2C::readRegister(const uint8_t reg_address, uint8_t &value)
{
	// Write register address to the sensor.
	int ret = transfer(&reg_address, sizeof(reg_address), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, &value, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int TFMINI_I2C::readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length)
{
	// Write register address to the sensor.
	int ret = transfer(&reg_address, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Read from the sensor.
	ret = transfer(nullptr, 0, &value[0], length);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}


void TFMINI_I2C::RunImpl()
{

	// mavlink_log_info(&mavlink_log_pub_user,"TFMINI_I2C_start: %d %.3f",i,(double)_collect_phase);


	PX4_INFO("RUN");
	measure();

	if (_collect_phase) {

		_collect_phase = false;
		_new_measurement = true;

		collect();

		ScheduleDelayed(TFMINI_I2C_SAMPLE_RATE);
	}

	count_s++;
	if(count_s>5){
	count_s=0;


		// mavlink_log_info(&mavlink_log_pub_user,"TFMINI_I2C: %.3f",(double)_collect_phase);

	}
}


int TFMINI_I2C::sensorInit()
{
	PX4_INFO("Init");

	uint8_t val = 0;

	// I2C at 2.8V on sensor side of level shifter
	int ret = PX4_OK;
	// ret |= readRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	// ret |= writeRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val | 0x01);

	// // set I2C to standard mode
	// ret |= writeRegister(0x88, 0x00);
	// ret |= writeRegister(0x80, 0x01);
	// ret |= writeRegister(0xFF, 0x01);
	// ret |= writeRegister(0x00, 0x00);
	// ret |= readRegister(0x91, val);
	// ret |= writeRegister(0x00, 0x01);
	// ret |= writeRegister(0xFF, 0x00);
	// ret |= writeRegister(0x80, 0x00);

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	_stop_variable = val;

	// Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	// readRegister(MSRC_CONFIG_CONTROL_REG, val);
	// writeRegister(MSRC_CONFIG_CONTROL_REG, val | 0x12);

	// // Set signal rate limit to 0.1
	// float rate_limit = 0.1 * 65536;
	// uint8_t rate_limit_split[2] = {};

	// rate_limit_split[0] = (((uint16_t)rate_limit) >> 8);
	// rate_limit_split[1] = (uint16_t)rate_limit;

	// writeRegisterMulti(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, &rate_limit_split[0], 2);
	// writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xFF);

	//spadCalculations();

	return PX4_OK;
}


int TFMINI_I2C::writeRegister(const uint8_t reg_address, const uint8_t value)
{
	uint8_t cmd[2] {reg_address, value};
	int ret = transfer(&cmd[0], 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

int TFMINI_I2C::writeRegisterMulti(const uint8_t reg_address, const uint8_t *value, const uint8_t length)
{
	if (length > 6 || length < 1) {
		DEVICE_LOG("VL53L0X::writeRegisterMulti length out of range");
		return PX4_ERROR;
	}

	/* be careful: for uint16_t to send first higher byte */
	uint8_t cmd[7] {};
	cmd[0] = reg_address;

	memcpy(&cmd[1], &value[0], length);

	int ret = transfer(&cmd[0], length + 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

void TFMINI_I2C::print_usage()
{
	PRINT_MODULE_USAGE_NAME("tfmini_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x10);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tfmini_i2c_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = TFMINI_I2C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_FORWARD_FACING;
	cli.i2c_address = TFMINI_BASEADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFMINI);

	if (!strcmp(verb, "start")) {
		PX4_INFO("TFmini_i2c START");

		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
