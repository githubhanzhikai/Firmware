#include "IEKF.hpp"
#include "matrix/filter.hpp"

static const float mag_inclination = 1.0f;
static const float mag_declination = 0;

IEKF::IEKF() :
	_nh(), // node handle
	_subImu(_nh.subscribe("sensor_combined", 0, &IEKF::callbackImu, this)),
	_subGps(_nh.subscribe("vehicle_gps_position", 0, &IEKF::correctGps, this)),
	_pubAttitude(_nh.advertise<vehicle_attitude_s>("vehicle_attitude", 0)),
	_pubLocalPosition(_nh.advertise<vehicle_local_position_s>("vehicle_local_position", 0)),
	_pubGlobalPosition(_nh.advertise<vehicle_global_position_s>("vehicle_global_position", 0)),
	_pubControlState(_nh.advertise<control_state_s>("control_state", 0)),
	_pubEstimatorStatus(_nh.advertise<estimator_status_s>("estimator_status", 0)),
	_x(),
	_P(),
	_u(),
	_g_n(0, 0, -9.8),
	_B_n(),
	_origin(),
	_timestampAccel(),
	_timestampMag(),
	_timestampBaro(),
	_timestampGps()
{
	// start with 0 quaternion
	_x(X::q_nb_0) = 1;
	_x(X::q_nb_1) = 0;
	_x(X::q_nb_2) = 0;
	_x(X::q_nb_3) = 0;

	// start with 1 accel scale
	_x(X::accel_scale) = 1;

	// initialize covariance
	_P(Xe::rot_n, Xe::rot_n) = 10;
	_P(Xe::rot_e, Xe::rot_e) = 10;
	_P(Xe::rot_d, Xe::rot_d) = 100;
	_P(Xe::vel_n, Xe::vel_n) = 1e9;
	_P(Xe::vel_e, Xe::vel_e) = 1e9;
	_P(Xe::vel_d, Xe::vel_d) = 1e9;
	_P(Xe::gyro_bias_n, Xe::gyro_bias_n) = 1e-3;
	_P(Xe::gyro_bias_e, Xe::gyro_bias_e) = 1e-3;
	_P(Xe::gyro_bias_d, Xe::gyro_bias_d) = 1e-3;
	_P(Xe::accel_scale, Xe::accel_scale) = 1e-1;
	_P(Xe::pos_n, Xe::pos_n) = 1e9;
	_P(Xe::pos_e, Xe::pos_e) = 1e9;
	_P(Xe::pos_d, Xe::pos_d) = 1e9;
	_P(Xe::terrain_alt, Xe::terrain_alt) = 1e9;
	_P(Xe::baro_bias, Xe::baro_bias) = 1e9;

	// initial magnetic field guess
	_B_n = Vector3f(0.21523, 0.00771, -0.42741);
}

Vector<float, X::n> IEKF::dynamics(const Vector<float, X::n> &x, const Vector<float, U::n> &u)
{
	Quaternion<float> q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3<float> a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3<float> as_n = q_nb.conjugate(a_b / _x(X::accel_scale)) - _g_n;
	Vector3<float> gyro_bias_b(_x(X::gyro_bias_bx), _x(X::gyro_bias_by), _x(X::gyro_bias_bz));
	Vector3<float> omega_nb_b(_u(U::omega_nb_bx), _u(U::omega_nb_by), _u(U::omega_nb_bz));
	Vector3<float> omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	Quaternion<float> dq_nb = q_nb * Quaternion<float>(0, omega_nb_b_corrected(0),
				  omega_nb_b_corrected(1), omega_nb_b_corrected(2)) * 0.5f;
	//ROS_INFO("a_b: %10.4f %10.4f %10.4f\n", double(a_b(0)), double(a_b(1)), double(a_b(2)));
	//ROS_INFO("as_n: %10.4f %10.4f %10.4f\n", double(as_n(0)), double(as_n(1)), double(as_n(2)));

	Vector<float, X::n> dx;
	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);
	dx(X::vel_n) = as_n(0);
	dx(X::vel_e) = as_n(1);
	dx(X::vel_d) = as_n(2);
	dx(X::gyro_bias_bx) = 0;
	dx(X::gyro_bias_by) = 0;
	dx(X::gyro_bias_bz) = 0;
	dx(X::accel_scale) = 0;
	dx(X::pos_n) = x(X::vel_n);
	dx(X::pos_e) = x(X::vel_e);
	dx(X::pos_d) = x(X::vel_d);
	dx(X::terrain_alt) = 0;
	dx(X::baro_bias) = 0;
	return dx;
}

void IEKF::callbackImu(const sensor_combined_s *msg)
{
	//ROS_INFO("imu callback");
	_u(U::omega_nb_bx) = msg->gyro_rad[0];
	_u(U::omega_nb_by) = msg->gyro_rad[1];
	_u(U::omega_nb_bz) = msg->gyro_rad[2];
	_u(U::accel_bx) = msg->accelerometer_m_s2[0];
	_u(U::accel_by) = msg->accelerometer_m_s2[1];
	_u(U::accel_bz) = msg->accelerometer_m_s2[2];

	// predict driven by gyro callback
	if (msg->gyro_integral_dt > 0) {
		predict(msg->gyro_integral_dt);
	};

	// correct  if new data
	correctAccel(msg);

	correctMag(msg);

	correctBaro(msg);

	publish();
}

void IEKF::correctAccel(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampAccelNew = msg->timestamp + msg->accelerometer_timestamp_relative;

	if (timestampAccelNew != _timestampAccel) {
		dt = (timestampAccelNew - _timestampAccel) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampAccel = timestampAccelNew;

	} else {
		return;
	}

	// measurement
	Vector3f y_b(
		msg->accelerometer_m_s2[0],
		msg->accelerometer_m_s2[1],
		msg->accelerometer_m_s2[2]);

	// don't correct if accelerating
	if (fabsf(Vector3<float>(y_b / _x(X::accel_scale)).norm() - _g_n.norm()) > 1.0f) {
		return;
	}

	// calculate residual
	Quaternion<float> q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
			       _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f r = q_nb.conjugate(y_b / _x(X::accel_scale)) - _g_n;

	// define R
	Matrix<float, Y_accel::n, Y_accel::n> R;
	R(Y_accel::accel_bx, Y_accel::accel_bx) = 1.0f / dt;
	R(Y_accel::accel_by, Y_accel::accel_by) = 1.0f / dt;
	R(Y_accel::accel_bz, Y_accel::accel_bz) = 1.0f / dt;

	// define H
	Matrix<float, Y_accel::n, Xe::n> H;
	Matrix3f tmp = _g_n.unit().hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_accel::accel_bx + i, Xe::rot_n + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_accel::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_accel::n]) {
		ROS_WARN("accel fault");
	}

	// don't allow yaw correction
	dxe(Xe::rot_d) = 0;

	applyErrorCorrection(dxe);
	setP(_P + dP);
}

void IEKF::correctMag(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampMagNew = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (timestampMagNew != _timestampMag) {
		dt = (timestampMagNew - _timestampMag) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampMag = timestampMagNew;

	} else {
		return;
	}

	// calculate residual
	Quaternion<float> q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
			       _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3<float> y_b = Vector3<float>(
				     msg->magnetometer_ga[0],
				     msg->magnetometer_ga[1],
				     msg->magnetometer_ga[2]).unit();
	Vector3<float> B_n = _B_n.unit();
	Vector3<float> r = q_nb.conjugate(y_b) - B_n;

	// define R
	Matrix<float, Y_mag::n, Y_mag::n> R;
	R(Y_mag::mag_n, Y_mag::mag_n) = 1.0f / dt;
	R(Y_mag::mag_e, Y_mag::mag_e) = 1.0f / dt;
	R(Y_mag::mag_d, Y_mag::mag_d) = 100.0f / dt; // don't want to correct roll/pitch

	// define H
	Matrix<float, Y_mag::n, Xe::n> H;
	Matrix3f tmp = B_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_mag::mag_n + i, Xe::rot_n + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_mag::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_mag::n]) {
		ROS_WARN("mag fault");
	}

	// don't allow roll/ pitch correction
	dxe(Xe::rot_n) = 0;
	dxe(Xe::rot_e) = 0;

	applyErrorCorrection(dxe);
	setP(_P + dP);
}

void IEKF::correctBaro(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampBaroNew = msg->timestamp + msg->baro_timestamp_relative;

	if (timestampBaroNew != _timestampBaro) {
		dt = (timestampBaroNew - _timestampBaro) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampBaro = timestampBaroNew;

	} else {
		return;
	}

	// calculate residual
	Vector<float, Y_baro::n> y;
	y(Y_baro::asl) = msg->baro_alt_meter;
	Vector<float, Y_baro::n> yh;
	yh(Y_baro::asl)	= -_x(X::pos_d) + _x(X::baro_bias) - _origin.getAlt();
	Vector<float, Y_baro::n> r = y - yh;

	// define R
	Matrix<float, Y_baro::n, Y_baro::n> R;
	R(Y_baro::asl, Y_baro::asl) = 10.0f / dt;

	// define H
	Matrix<float, Y_baro::n, Xe::n> H;
	H(Y_baro::asl, Xe::pos_d) = -1;
	H(Y_baro::asl, Xe::baro_bias) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_baro::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_baro::n]) {
		ROS_WARN("baro fault");
	}

	applyErrorCorrection(dxe);
	setP(_P + dP);

}

void IEKF::correctGps(const vehicle_gps_position_s *msg)
{
	// check for good gps signal
	if (msg->satellites_used < 6 || msg->fix_type < 3) {
		return;
	}

	_timestampGps = msg->timestamp;
	double lat_deg = msg->lat * 1e-7;
	double lon_deg = msg->lon * 1e-7;
	float alt_m = msg->alt * 1e-3;

	// init global reference
	if (!_origin.xyInitialized()) {
		ROS_INFO("gps map ref init %12.6f %12.6f", double(lat_deg), double(lon_deg));
		_origin.xyInitialize(lat_deg, lon_deg, msg->timestamp);
	}

	if (!_origin.altInitialized()) {
		ROS_INFO("gps alt init %12.2f", double(alt_m));
		_origin.altInitialize(alt_m, msg->timestamp);
	}

	// calculate residual
	float gps_pos_n = 0;
	float gps_pos_e = 0;
	float gps_pos_d = 0;
	_origin.globalToLocal(lat_deg, lon_deg, alt_m,
			      gps_pos_n, gps_pos_e, gps_pos_d);

	Vector<float, Y_gps::n> y;
	y(Y_gps::pos_n) = gps_pos_n;
	y(Y_gps::pos_e) = gps_pos_e;
	y(Y_gps::pos_d) = gps_pos_d;
	y(Y_gps::vel_n) = msg->vel_n_m_s;
	y(Y_gps::vel_e) = msg->vel_e_m_s;
	y(Y_gps::vel_d) = msg->vel_d_m_s;

	Vector<float, Y_gps::n> yh;
	yh(Y_gps::pos_n) = _x(X::pos_n);
	yh(Y_gps::pos_e) = _x(X::pos_e);
	yh(Y_gps::pos_d) = _x(X::pos_d);
	yh(Y_gps::vel_n) = _x(X::vel_n);
	yh(Y_gps::vel_e) = _x(X::vel_e);
	yh(Y_gps::vel_d) = _x(X::vel_d);

	Vector<float, Y_gps::n> r = y - yh;

	// define R
	Matrix<float, Y_gps::n, Y_gps::n> R;
	R(Y_gps::pos_n, Y_gps::pos_n) = 1;
	R(Y_gps::pos_e, Y_gps::pos_e) = 1;
	R(Y_gps::pos_d, Y_gps::pos_d) = 1;
	R(Y_gps::vel_n, Y_gps::vel_n) = 1;
	R(Y_gps::vel_e, Y_gps::vel_e) = 1;
	R(Y_gps::vel_d, Y_gps::vel_d) = 1;

	// define H
	Matrix<float, Y_gps::n, Xe::n> H;
	H(Y_gps::pos_n, Xe::pos_n) = 1;
	H(Y_gps::pos_e, Xe::pos_e) = 1;
	H(Y_gps::pos_d, Xe::pos_d) = 1;
	H(Y_gps::vel_n, Xe::vel_n) = 1;
	H(Y_gps::vel_e, Xe::vel_e) = 1;
	H(Y_gps::vel_d, Xe::vel_d) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_gps::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_gps::n]) {
		ROS_WARN("gps fault");
	}

	//ROS_INFO("gps rot correct %10.4f %10.4f %10.4f",
	//double(dxe(Xe::rot_n)),
	//double(dxe(Xe::rot_e)),
	//double(dxe(Xe::rot_d)));
	dxe(Xe::rot_n) = 0;
	dxe(Xe::rot_e) = 0;
	dxe(Xe::rot_d) = 0;

	applyErrorCorrection(dxe);
	setP(_P + dP);
}

void IEKF::predict(float dt)
{
	// define process noise matrix
	Matrix<float, Xe::n, Xe::n> Q;
	Q(Xe::rot_n, Xe::rot_n) = 1e-1;
	Q(Xe::rot_e, Xe::rot_e) = 1e-1;
	Q(Xe::rot_d, Xe::rot_d) = 1e-1;
	Q(Xe::vel_n, Xe::vel_n) = 1e-1;
	Q(Xe::vel_e, Xe::vel_e) = 1e-1;
	Q(Xe::vel_d, Xe::vel_d) = 1e-1;
	Q(Xe::gyro_bias_n, Xe::gyro_bias_n) = 1e-4;
	Q(Xe::gyro_bias_e, Xe::gyro_bias_e) = 1e-4;
	Q(Xe::gyro_bias_d, Xe::gyro_bias_d) = 1e-4;
	Q(Xe::accel_scale, Xe::accel_scale) = 1e-2;
	Q(Xe::pos_n, Xe::pos_n) = 1e-1;
	Q(Xe::pos_e, Xe::pos_e) = 1e-1;
	Q(Xe::pos_d, Xe::pos_d) = 1e-1;
	Q(Xe::terrain_alt, Xe::terrain_alt) = 1e-1f;
	Q(Xe::baro_bias, Xe::baro_bias) = 1e-1f;

	// define A matrix
	Matrix<float, Xe::n, Xe::n> A;

	// derivative of rotation error is -0.5 * gyro bias
	A(Xe::rot_n, Xe::Xe::gyro_bias_n) = -0.5;
	A(Xe::rot_e, Xe::Xe::gyro_bias_e) = -0.5;
	A(Xe::rot_d, Xe::Xe::gyro_bias_d) = -0.5;

	// derivative of velocity
	Quaternion<float> q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));

	if (fabsf(q_nb.norm() - 1.0f) > 1e-3f) {
		ROS_INFO("normalizing quaternion, norm was %6.4f\n", double(q_nb.norm()));
		q_nb.normalize();
		_x(X::q_nb_0) = q_nb(0);
		_x(X::q_nb_1) = q_nb(1);
		_x(X::q_nb_2) = q_nb(2);
		_x(X::q_nb_3) = q_nb(3);
	}

	Vector3<float> a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3<float> J_a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Matrix<float, 3, 3> a_tmp = -J_a_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::vel_n + i, Xe::rot_n + j) = a_tmp(i, j);
		}

		A(Xe::vel_n + i, Xe::accel_scale) = -J_a_n(i);
	}

	// derivative of gyro bias
	Vector3<float> omega_nb_b(
		_u(U::omega_nb_bx), _u(U::omega_nb_by), _u(U::omega_nb_bz));
	Vector3<float> gyro_bias_b(
		_x(X::gyro_bias_bx), _x(X::gyro_bias_by), _x(X::gyro_bias_bz));
	Vector3<float> J_omega_n = q_nb.conjugate(omega_nb_b - gyro_bias_b);
	Matrix<float, 3, 3> g_tmp = J_omega_n.hat();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::gyro_bias_n + i, Xe::rot_n + j) = g_tmp(i, j);
		}
	}

	// derivative of position is velocity
	A(Xe::pos_n, Xe::vel_n) = 1;
	A(Xe::pos_e, Xe::vel_e) = 1;
	A(Xe::pos_d, Xe::vel_d) = 1;

	// derivative of terrain alt is zero

	// derivative of baro bias is zero

	//ROS_INFO("A:");
	//for (int i=0;i<Xe::n; i++) {
	//for (int j=0;j<Xe::n; j++) {
	//printf("%10.3f, ", double(A(i, j)));
	//}
	//printf("\n");
	//}

	// propgate state using euler integration
	Vector<float, X::n> dx = dynamics(_x, _u) * dt;
	//ROS_INFO("dx predict \n");
	//dx.print();
	_x = _x + dx;
	boundX();

	// propgate covariance using euler integration
	Matrix<float, Xe::n, Xe::n> dP = (A * _P + _P * A.T() + Q) * dt;
	setP(_P + dP);

	//ROS_INFO("P:");
	//_P.print();

}

void IEKF::applyErrorCorrection(const Vector<float, Xe::n> &d_xe)
{
	Quaternion<float> q_nb(_x(X::q_nb_0), _x(X::q_nb_1), _x(X::q_nb_2), _x(X::q_nb_3));
	Quaternion<float> d_q_nb = Quaternion<float>(0,
				   d_xe(Xe::rot_n), d_xe(Xe::rot_e), d_xe(Xe::rot_d)) * q_nb;
	//ROS_INFO("d_q_nb");
	//d_q_nb.print();
	Vector3<float> d_gyro_bias_b = q_nb.conjugate_inversed(
					       Vector3<float>(d_xe(Xe::gyro_bias_n),
							       d_xe(Xe::gyro_bias_e),
							       d_xe(Xe::gyro_bias_d)));

	// linear term correction is the same
	// as the error correction
	Vector<float, X::n> dx;
	dx(X::q_nb_0) = d_q_nb(0);
	dx(X::q_nb_1) = d_q_nb(1);
	dx(X::q_nb_2) = d_q_nb(2);
	dx(X::q_nb_3) = d_q_nb(3);
	dx(X::vel_n) = d_xe(Xe::vel_n);
	dx(X::vel_e) = d_xe(Xe::vel_e);
	dx(X::vel_d) = d_xe(Xe::vel_d);
	dx(X::gyro_bias_bx) = d_gyro_bias_b(0);
	dx(X::gyro_bias_by) = d_gyro_bias_b(1);
	dx(X::gyro_bias_bz) = d_gyro_bias_b(2);
	dx(X::accel_scale) = _x(X::accel_scale) * d_xe(Xe::accel_scale);
	dx(X::pos_n) = d_xe(Xe::pos_n);
	dx(X::pos_e) = d_xe(Xe::pos_e);
	dx(X::pos_d) = d_xe(Xe::pos_d);
	dx(X::terrain_alt) = d_xe(Xe::terrain_alt);
	dx(X::baro_bias) = d_xe(Xe::baro_bias);

	//ROS_INFO("dx correct \n");
	//dx.print();
	_x = _x + dx;
	boundX();
}

void IEKF::setP(const SquareMatrix<float, Xe::n> &P)
{
	_P = P;

	for (int i = 0; i < Xe::n; i++) {
		// only operate on upper triangle, then copy to lower

		// don't allow NaN or large numbers
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				ROS_INFO("P(%d, %d) NaN, setting to 0", i, j);
				_P(i, j) = 0;
			}

			if (_P(i, j) > 1e9f) {
				// upper bound
				_P(i, j) = 1e9;
			}
		}

		// force positive diagonal
		if (_P(i, i) < 1e-6f) {
			ROS_INFO("P(%d, %d) < 1e-6, setting to 1e-6", i, i);
			_P(i, i) = 1.0e-6f;
		}

		// force symmetry, copy uppper triangle to lower
		for (int j = 0; j < i; j++) {
			_P(j, i) = _P(i, j);
		}
	}
}

void IEKF::boundX()
{
	// for quaterinons we bound at 2
	// so that saturation doesn't change
	// the direction of the vectors typicall
	// and normalization
	// handles small errors
	Vector<float, X::n> lowerBound;
	lowerBound(X::q_nb_0) = -2;
	lowerBound(X::q_nb_1) = -2;
	lowerBound(X::q_nb_2) = -2;
	lowerBound(X::q_nb_3) = -2;
	lowerBound(X::vel_n) = -100;
	lowerBound(X::vel_e) = -100;
	lowerBound(X::vel_d) = -100;
	lowerBound(X::gyro_bias_bx) = 0;
	lowerBound(X::gyro_bias_by) = 0;
	lowerBound(X::gyro_bias_bz) = 0;
	lowerBound(X::accel_scale) = 0.8;
	lowerBound(X::pos_n) = -1e9;
	lowerBound(X::pos_e) = -1e9;
	lowerBound(X::pos_d) = -1e9;
	lowerBound(X::terrain_alt) = -1e6;
	lowerBound(X::baro_bias) = -1e6;

	Vector<float, X::n> upperBound;
	upperBound(X::q_nb_0) = 2;
	upperBound(X::q_nb_1) = 2;
	upperBound(X::q_nb_2) = 2;
	upperBound(X::q_nb_3) = 2;
	upperBound(X::vel_n) = 100;
	upperBound(X::vel_e) = 100;
	upperBound(X::vel_d) = 100;
	upperBound(X::gyro_bias_bx) = 0;
	upperBound(X::gyro_bias_by) = 0;
	upperBound(X::gyro_bias_bz) = 0;
	upperBound(X::accel_scale) = 1.5;
	upperBound(X::pos_n) = 1e9;
	upperBound(X::pos_e) = 1e9;
	upperBound(X::pos_d) = 1e9;
	upperBound(X::terrain_alt) = 1e6;
	upperBound(X::baro_bias) = 1e6;

	for (int i = 0; i < X::n; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			ROS_INFO("x(%d) NaN, setting to 0", i);
			_x(i) = 0.0;
		}

		if (_x(i) < lowerBound(i)) {
			//ROS_INFO("x(%d) < lower bound, saturating", i);
			_x(i) = lowerBound(i);

		} else if (_x(i) > upperBound(i)) {
			//ROS_INFO("x(%d) > upper bound, saturating", i);
			_x(i) = upperBound(i);
		}
	}
}

void IEKF::publish()
{
	//ROS_INFO("x:");
	//_x.print();

	//ROS_INFO("P:");
	//_P.diag().print();

	float eph = sqrt(_P(Xe::pos_n, Xe::pos_n) + _P(Xe::pos_e, Xe::pos_e));
	float epv = _P(Xe::pos_d, Xe::pos_d);
	Quaternion<float> q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Euler<float> euler_nb = q_nb;
	Vector3<float> a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3<float> a_spec_b = a_b / _x(X::accel_scale) - q_nb.conjugate_inversed(_g_n);
	ros::Time now = ros::Time::now();

	// publish attitude
	{
		vehicle_attitude_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.rollspeed = _u(U::omega_nb_bx) - _x(X::gyro_bias_bx);
		msg.pitchspeed = _u(U::omega_nb_by) - _x(X::gyro_bias_by);
		msg.yawspeed = _u(U::omega_nb_bz) - _x(X::gyro_bias_bz);
		_pubAttitude.publish(msg);
	}

	// publish local position
	{
		vehicle_local_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.xy_valid = true;
		msg.z_valid = true;
		msg.v_xy_valid = true;
		msg.v_z_valid = true;
		msg.x = _x(X::pos_n);
		msg.y = _x(X::pos_e);
		msg.z = _x(X::pos_d);
		msg.delta_xy[0] = 0;
		msg.delta_xy[1] = 0;
		msg.delta_z = 0;
		msg.vx = _x(X::vel_n);
		msg.vy = _x(X::vel_e);
		msg.vz = _x(X::vel_d);
		msg.delta_vxy[0] = 0;
		msg.delta_vxy[1] = 0;
		msg.delta_vz = 0;
		msg.xy_reset_counter = 0;
		msg.z_reset_counter = 0;
		msg.vxy_reset_counter = 0;
		msg.vz_reset_counter = 0;
		msg.yaw = euler_nb(2);
		msg.xy_global = _origin.xyInitialized();
		msg.z_global = _origin.altInitialized();
		msg.ref_timestamp = _origin.getXYTimestamp();
		msg.ref_lat = _origin.getLatDeg();
		msg.ref_lon = _origin.getLonDeg();
		msg.ref_alt = _origin.getAlt();
		msg.dist_bottom = -_x(X::pos_d) - _x(X::terrain_alt);
		msg.dist_bottom_rate = -_x(X::vel_d);
		msg.surface_bottom_timestamp = 0;
		msg.dist_bottom_valid = true;
		msg.eph = eph;
		msg.epv = epv;
		_pubLocalPosition.publish(msg);
	}

	// publish global position
	{
		double lat_deg = 0;
		double lon_deg = 0;
		float alt_m = 0;
		_origin.localToGlobal(_x(X::pos_n), _x(X::pos_e), _x(X::pos_d), lat_deg, lon_deg, alt_m);
		vehicle_global_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.time_utc_usec = 0; // TODO
		msg.lat = lat_deg;
		msg.lon = lon_deg;
		msg.alt = alt_m;
		msg.delta_lat_lon[0] = 0;
		msg.delta_lat_lon[1] = 0;
		msg.delta_alt = 0;
		msg.lat_lon_reset_counter = 0;
		msg.alt_reset_counter = 0;
		msg.vel_n = _x(X::vel_n);
		msg.vel_e = _x(X::vel_e);
		msg.vel_d = _x(X::vel_d);
		msg.yaw = euler_nb(2);
		msg.eph = eph;
		msg.epv = epv;
		msg.terrain_alt = _x(X::terrain_alt) + _origin.getAlt();
		msg.terrain_alt_valid = true;
		msg.dead_reckoning = false;
		msg.pressure_alt = alt_m; // TODO
		_pubGlobalPosition.publish(msg);
	}

	// publish control state
	{
		// specific acceleration
		control_state_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.x_acc = a_spec_b(0);
		msg.y_acc = a_spec_b(1);
		msg.z_acc = a_spec_b(2);
		msg.x_vel = _x(X::vel_n);
		msg.y_vel = _x(X::vel_e);
		msg.z_vel = _x(X::vel_d);
		msg.x_pos = _x(X::pos_n);
		msg.y_pos = _x(X::pos_e);
		msg.z_pos = _x(X::pos_d);
		msg.airspeed = 0;
		msg.airspeed_valid = false;
		msg.vel_variance[0] = _P(Xe::vel_n, Xe::vel_n);
		msg.vel_variance[1] = _P(Xe::vel_e, Xe::vel_e);
		msg.vel_variance[2] = _P(Xe::vel_d, Xe::vel_d);
		msg.pos_variance[0] = _P(Xe::pos_n, Xe::pos_n);
		msg.pos_variance[1] = _P(Xe::pos_e, Xe::pos_e);
		msg.pos_variance[2] = _P(Xe::pos_d, Xe::pos_d);
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.delta_q_reset[0] = 0;
		msg.delta_q_reset[1] = 0;
		msg.delta_q_reset[2] = 0;
		msg.delta_q_reset[3] = 0;
		msg.quat_reset_counter = 0;
		msg.roll_rate = _u(U::omega_nb_bx) - _x(X::gyro_bias_bx);
		msg.pitch_rate = _u(U::omega_nb_by) - _x(X::gyro_bias_by);
		msg.yaw_rate = _u(U::omega_nb_bz) - _x(X::gyro_bias_bz);
		msg.horz_acc_mag = 0;
		_pubControlState.publish(msg);
	}

	// estimator status
	{
		estimator_status_s msg = {};
		msg.timestamp = now.toNSec();
		msg.vibe[0] = 0; // TODO
		msg.vibe[1] = 0; // TODO
		msg.vibe[2] = 0; // TODO
		msg.n_states = X::n;

		for (int i = 0; i < X::n; i++) {
			msg.states[i] = _x(i);
			// offset by 1 so covariances lined up with states
			// except for quaternoin and rotations error
		}

		for (int i = 0; i < Xe::n; i++) {
			msg.covariances[i] = _P(i, i);
		}

		msg.gps_check_fail_flags = 0; // TODO
		msg.control_mode_flags = 0; // TODO
		msg.filter_fault_flags = 0; // TODO
		msg.pos_horiz_accuracy = eph;
		msg.pos_vert_accuracy = epv;
		msg.innovation_check_flags = 0; // TODO
		msg.mag_test_ratio = 0; // TODO
		msg.vel_test_ratio = 0; // TODO
		msg.pos_test_ratio = 0; // TODO
		msg.hgt_test_ratio = 0; // TODO
		msg.tas_test_ratio = 0; // TODO
		msg.hagl_test_ratio = 0; // TODO
		msg.solution_status_flags = 0; // TODO
		_pubEstimatorStatus.publish(msg);
	}
}
