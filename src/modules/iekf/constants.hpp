#pragma once

const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;

/**
 * Note that structs are used instead of enums
 * to allow use in arrays without casting
 * and to keep size small
 */

/**
 * State enum
 */
struct X {
	static const uint8_t q_nb_0 = 0;
	static const uint8_t q_nb_1 = 1;
	static const uint8_t q_nb_2 = 2;
	static const uint8_t q_nb_3 = 3;
	static const uint8_t vel_n = 4;
	static const uint8_t vel_e = 5;
	static const uint8_t vel_d = 6;
	static const uint8_t gyro_bias_bx = 7;
	static const uint8_t gyro_bias_by = 8;
	static const uint8_t gyro_bias_bz = 9;
	static const uint8_t accel_scale = 10;
	static const uint8_t pos_n = 11;
	static const uint8_t pos_e = 12;
	static const uint8_t pos_d = 13;
	static const uint8_t terrain_alt = 14;
	static const uint8_t baro_bias = 15;
	static const uint8_t n = 16;
};

/**
 * Error state enum
 * used for linearization
 *
 * Note gyro bias in navigation frame
 */
struct Xe {
	static const uint8_t rot_n = 0;
	static const uint8_t rot_e = 1;
	static const uint8_t rot_d = 2;
	static const uint8_t vel_n = 3;
	static const uint8_t vel_e = 4;
	static const uint8_t vel_d = 5;
	static const uint8_t gyro_bias_n = 6;
	static const uint8_t gyro_bias_e = 7;
	static const uint8_t gyro_bias_d = 8;
	static const uint8_t accel_scale = 9;
	static const uint8_t pos_n = 10;
	static const uint8_t pos_e = 11;
	static const uint8_t pos_d = 12;
	static const uint8_t terrain_alt = 13;
	static const uint8_t baro_bias = 14;
	static const uint8_t n = 15;
};

/**
 * Input enum
 */
struct U {
	static const uint8_t omega_nb_bx = 0;
	static const uint8_t omega_nb_by = 1;
	static const uint8_t omega_nb_bz = 2;
	static const uint8_t accel_bx = 3;
	static const uint8_t accel_by = 4;
	static const uint8_t accel_bz = 5;
	static const uint8_t n = 6;
};

/**
 * Accel measurement enum
 */
struct Y_accel {
	static const uint8_t accel_bx = 0;
	static const uint8_t accel_by = 1;
	static const uint8_t accel_bz = 2;
	static const uint8_t n = 3;
};

/**
 * GPS measurement
 */
struct Y_gps {
	static const uint8_t pos_n = 0;
	static const uint8_t pos_e = 1;
	static const uint8_t pos_d = 2;
	static const uint8_t vel_n = 3;
	static const uint8_t vel_e = 4;
	static const uint8_t vel_d = 5;
	static const uint8_t n = 6;
};

/**
 * Baro measurement
 */
struct Y_baro {
	static const uint8_t asl = 0;
	static const uint8_t n = 1;
};

/**
 * Magnetometer measurement
 *
 * The filter treats the error
 * in the navigation frame
 * (north, east, down) even though the
 * field is measured in the body
 * frame.
 */
struct Y_mag {
	static const uint8_t mag_n = 0;
	static const uint8_t mag_e = 1;
	static const uint8_t mag_d = 2;
	static const uint8_t n = 3;
};

static const float BETA_TABLE[] = {
	0,
	8.82050518214,
	12.094592431,
	13.9876612368,
	16.0875642296,
	17.8797700658,
	19.6465647819,
	21.3802576894,
	23.0806434845,
	24.6673803845,
	26.1487953661,
	27.6350821245,
	29.6565383703,
	31.2211113844,
	32.7673547211,
	34.2967756977,
	35.6906782236,
	37.0724753352,
	38.4549693067,
	39.836592699,
};
