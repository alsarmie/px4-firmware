#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include <drivers/drv_hrt.h>

// mavlink pub
extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		REQ_FLOW_INIT_COUNT = 10;
static const uint32_t		FLOW_TIMEOUT = 1000000;	// 1 s

// minimum flow altitude
static const float flow_min_agl = 0.05;

#define PRFX "[lpe.flow] "
#define RATE_LIMIT(interval, time) (((time) + (uint64_t)(interval)*1.0e6f < hrt_absolute_time()) ? (time) = hrt_absolute_time() : 0)

void BlockLocalPositionEstimator::flowInit()
{
	// measure
	Vector<float, n_y_flow> y;

	if (flowMeasure(y) != OK) {
		_flowQStats.reset();
		return;
	}

	// if finished
	if (_flowQStats.getCount() > REQ_FLOW_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, PRFX "init, "
		                             "quality: %d, std: %d",
		                             int(_flowQStats.getMean()(0)),
		                             int(_flowQStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_FLOW;
		_sensorFault &= ~SENSOR_FLOW;
	}
}

int BlockLocalPositionEstimator::flowMeasure(Vector<float, n_y_flow> &y)
{
	float qual = _sub_flow.get().quality;

#if 0
	// check for sane pitch/roll
	if (_eul(0) > 0.5f || _eul(1) > 0.5f) {
		if (RATE_LIMIT(5.0f, _time_warn_flow)) {
			mavlink_and_console_log_info(&mavlink_log_pub, PRFX "attitude too far from level");
		}
		return -1;
	}

	// check quality
	if (qual < _flow_min_q.get()) {
		if (RATE_LIMIT(5.0f, _time_warn_flow)) {
			mavlink_and_console_log_info(&mavlink_log_pub, PRFX "flow quality below threshold");
		}
		return -1;
	}

	// check for agl
	if (agl() < flow_min_agl) {
		if (RATE_LIMIT(5.0f, _time_warn_flow)) {
			mavlink_and_console_log_info(&mavlink_log_pub, PRFX "agl too low");
		}
		return -1;
	}
#endif

#if 0
	// calculate range to center of image for flow
	if (!(_estimatorInitialized & EST_TZ)) {
		if (RATE_LIMIT(5.0f, _time_warn_flow)) {
			mavlink_and_console_log_info(&mavlink_log_pub, PRFX "terrain height not known, cannot determine AGL");
		}
		return -1;
	}
#endif

	matrix::Eulerf euler = matrix::Quatf(_sub_att.get().q);
	float d = math::max(flow_min_agl, agl()) * cosf(euler.phi()) * cosf(euler.theta());

	// optical flow in x, y axis
	// TODO consider making flow scale a states of the kalman filter
	float flow_x_rad = _sub_flow.get().pixel_flow_x_integral * _flow_scale.get();
	float flow_y_rad = _sub_flow.get().pixel_flow_y_integral * _flow_scale.get();
	float dt_flow    = _sub_flow.get().integration_timespan / 1.0e6f;

	if (dt_flow > 0.5f || dt_flow < 1.0e-6f) {
		return -1;
	}

	if (_fusion.get() & FUSE_FLOW_GYRO_COMP) {
#define LPE_FLOW_GYRO_MAIN
#ifdef LPE_FLOW_GYRO_MAIN
		flow_x_rad -= _sub_sensor.get().gyro_rad[0]*dt_flow;
		flow_y_rad -= _sub_sensor.get().gyro_rad[1]*dt_flow;
#else
		flow_x_rad -= _flow_gyro_x_high_pass.update(_sub_flow.get().gyro_x_rate_integral);
		flow_y_rad -= _flow_gyro_y_high_pass.update(_sub_flow.get().gyro_y_rate_integral);
#endif
	}
	//warnx("flow x: %10.4f y: %10.4f gyro_x: %10.4f gyro_y: %10.4f d: %10.4f",
	//double(flow_x_rad), double(flow_y_rad), double(gyro_x_rad), double(gyro_y_rad), double(d));

	// compute velocities in body frame using ground distance
	// note that the integral rates in the optical_flow uORB topic are RH
	// rotations about body axes
	Vector3f delta_b(
		+d*flow_y_rad,
		-d*flow_x_rad,
		0);

	// rotation of flow from body to nav frame
	Vector3f delta_n = _R_att * delta_b;
	//Vector3f delta_n = delta_b;

	// imporant to timestamp flow even if distance is bad
	_time_last_flow = _timeStamp;

	// measurement
	y(Y_flow_vx) = delta_n(0) / dt_flow;
	y(Y_flow_vy) = delta_n(1) / dt_flow;

	_flowQStats.update(Scalarf(qual));

	return OK;
}

void BlockLocalPositionEstimator::flowCorrect()
{
	// measure flow
	Vector<float, n_y_flow> y;

	if (flowMeasure(y) != OK) { return; }

	// flow measurement matrix and noise matrix
	Matrix<float, n_y_flow, n_x> C;
	C.setZero();
	C(Y_flow_vx, X_vx) = 1;
	C(Y_flow_vy, X_vy) = 1;

	SquareMatrix<float, n_y_flow> R;
	R.setZero();
	R(Y_flow_vx, Y_flow_vx) = _flow_p_stddev.get();
	R(Y_flow_vy, Y_flow_vy) = _flow_p_stddev.get();

	// residual
	Vector<float, n_y_flow> r = y - C * _x;

	// residual covariance
	Matrix<float, n_y_flow, n_y_flow> S = C * _P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().flow_innov[0] = r(0);
	_pub_innov.get().flow_innov[1] = r(1);
	_pub_innov.get().flow_innov_var[0] = S(0, 0);
	_pub_innov.get().flow_innov_var[1] = S(1, 1);

	// residual covariance, (inverse)
	Matrix<float, n_y_flow, n_y_flow> S_I = inv<float, n_y_flow>(S);

#if 0
	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_flow]) {
		if (!(_sensorFault & SENSOR_FLOW)) {
			mavlink_and_console_log_info(&mavlink_log_pub, PRFX "fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_FLOW;
		}
	} else if (_sensorFault & SENSOR_FLOW) {
		_sensorFault &= ~SENSOR_FLOW;
		mavlink_and_console_log_info(&mavlink_log_pub, PRFX "OK");
	}
#endif

	if (!(_sensorFault & SENSOR_FLOW)) {
		Matrix<float, n_x, n_y_flow> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		//Vector<float, n_x> dx = C.transpose() * r;
		_x += dx;
		_P -= K * C * _P;
	}

#if 0
	_sensorFault &= ~SENSOR_FLOW;
	_x(X_vx) = y(Y_flow_vx);
	_x(X_vy) = y(Y_flow_vy);
#endif
}

void BlockLocalPositionEstimator::flowCheckTimeout()
{
	if (_timeStamp - _time_last_flow > FLOW_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_FLOW)) {
			_sensorTimeout |= SENSOR_FLOW;
			_flowQStats.reset();
			mavlink_log_critical(&mavlink_log_pub, PRFX "timeout");
		}
	}
}

#undef PRFX
