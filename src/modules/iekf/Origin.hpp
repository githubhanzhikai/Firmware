#pragma once
#include <lib/geo/geo.h>
#include "constants.hpp"

class Origin
{
public:
	Origin() :
		_map_ref(),
		_alt(0),
		_altInitialized(false),
		_altTimestamp(0)
	{
	}
	inline bool altInitialized()
	{
		return _altInitialized;
	}

	inline bool xyInitialized()
	{
		return _map_ref.init_done;
	}

	inline uint64_t getXYTimestamp()
	{
		return _map_ref.timestamp;
	}

	inline bool getAltTimestamp()
	{
		return _altTimestamp;
	}

	void globalToLocal(const double &lat_deg, const double &lon_deg, const float &alt, float &pos_n, float &pos_e,
			   float &pos_d)
	{
		map_projection_project(&_map_ref, lat_deg, lon_deg, &pos_n, &pos_e);
		pos_d = -(alt - _alt);
	}

	void localToGlobal(const float &pos_n, const float &pos_e, const float &pos_d, double &lat_deg, double &lon_deg,
			   float &alt)
	{
		map_projection_reproject(&_map_ref, pos_n, pos_e, &lat_deg, &lon_deg);
		alt = -pos_d + _alt;
	}

	void xyInitialize(double lat_deg, double lon_deg, uint64_t timestamp)
	{
		map_projection_init_timestamped(&_map_ref,
						lat_deg, lon_deg, timestamp);
	}
	inline float getAlt()
	{
		return _alt;
	}
	void altInitialize(float alt, uint64_t timestamp)
	{
		_alt = alt;
		_altInitialized = true;
		_altTimestamp = timestamp;
	}
	inline double getLatRad()
	{
		return _map_ref.lat_rad;
	}
	inline double getLonRad()
	{
		return _map_ref.lon_rad;
	}
	inline double getLatDeg()
	{
		return rad2deg * getLatRad();
	}
	inline double getLonDeg()
	{
		return rad2deg * getLonRad();
	}
private:
	struct map_projection_reference_s _map_ref;
	float _alt;
	bool _altInitialized;
	uint64_t _altTimestamp;
};
