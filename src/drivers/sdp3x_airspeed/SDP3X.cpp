/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "SDP3X.hpp"

int
SDP3X::measure()
{
	// This sensor is operated in continous measure mode
	int ret = PX4_ERROR;

	if (!_inited) {
		_inited = init_sdp3x();
		ret = !_inited;
	}

	return ret;
}

bool
SDP3X::init_sdp3x()
{
	uint8_t cmd[4];
	uint8_t val[18];

	// Step 1 - reset
	cmd[1] = 0x3F;
	cmd[0] = 0xF9;
	int ret = transfer(&cmd[0], 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_ERR("reset failed");
		return false;
	}

	usleep(3000);

	// ID
	cmd[0] = 0x36;//36
	cmd[1] = 0x7c;//7c
	cmd[2] = 0xe1;//e1
	cmd[3] = 0x02;//02
	ret = transfer(&cmd[0], 4, &val[0], 18);

	if (ret == PX4_OK) {
		PX4_INFO("request ID value cmd: %X ret: %02X%02X%02X%02X", (unsigned char)cmd[0],
			 (unsigned char)val[0], (unsigned char)val[1], (unsigned char)val[3], (unsigned char)val[4]);

	} else {
		perf_count(_comms_errors);
		return false;
	}

	// Step 2 - start continous differential mode
	cmd[0] = 0x36;
	cmd[1] = 0x15;

	ret = transfer(&cmd[0], 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return false;
	}

	usleep(8000);

	// read 9 byte complete value
	ret = transfer(nullptr, 0, &val[0], 9);

	if (ret == PX4_OK) {
		PX4_INFO("request value cmd: %X ret: %04X", cmd, val[0]);

	} else {
		perf_count(_comms_errors);
		return false;
	}

	return true;
}

int
SDP3X::collect()
{
	perf_begin(_sample_perf);

	// read 9 bytes from the sensor
	uint8_t val[9];
	int ret = transfer(nullptr, 0, &val[0], 9);

	if (ret != PX4_OK) {
		return ret;
	}

	// Check the CRC
	if (1 == 2) {
		perf_count(_comms_errors);
		return EAGAIN;
	}

	int32_t P = 0;

	const float diff_press_PSI = P * 0.0001f;

	// 1 PSI = 6894.76 Pascals
	const float PSI_to_Pa = 6894.757f;
	float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
	float temperature_c = 0.0f;

	//PX4_WARN("T: %.1f PSI: %.3f Pa: %.3f", (double)temperature_c, (double)diff_press_PSI, (double)diff_press_pa_raw);

	// the raw value still should be compensated for the known offset
	diff_press_pa_raw -= _diff_pres_offset;

	differential_pressure_s report;

	/* track maximum differential pressure measured (so we can work out top speed). */
	if (diff_press_pa_raw > _max_differential_pressure_pa) {
		_max_differential_pressure_pa = diff_press_pa_raw;
	}

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature_c;
	report.differential_pressure_filtered_pa =  _filter.apply(diff_press_pa_raw);
	report.differential_pressure_raw_pa = diff_press_pa_raw;
	report.max_differential_pressure_pa = _max_differential_pressure_pa;

	if (_airspeed_pub != nullptr && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(differential_pressure), _airspeed_pub, &report);
	}

	new_report(report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
SDP3X::cycle()
{
	int ret = PX4_ERROR;

	/* measurement phase */
	ret = collect();

	if (PX4_OK != ret) {
		_sensor_ok = false;
		DEVICE_DEBUG("measure error");
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&Airspeed::cycle_trampoline, this, USEC2TICK(CONVERSION_INTERVAL));
}
