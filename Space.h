#pragma once
#include <math.h>
#include "DataStore.h"
#define PI 3.1415926535897932384626433832795
#define PI_d_2 (PI/2)

#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2     frequency (Hz) */

struct BLH;
struct XYZ;
struct ENU;

struct Coordinate{
	double a;
	double f;
};
#define WGS_84 {6378137.0,1.0/298.257223563}

struct BLH {
	double B, L, H;
	//void toXYZ(XYZ * out, const Coordinate * coor);
	//void toXYZ(XYZ * out);
};
struct ENU {
	double E, N, U;
};
struct XYZ{
	double X, Y, Z;
	//void toBLH(BLH * out, const Coordinate * coor);
	void toBLH(BLH * out) // using WGS_84 as the coordinate
	{
		const static double a = 6378137.0;
		const static double F = 1.0 / 298.257223563;
		const XYZ * xyz = this;
		double e2, Z, dZ, ZdZ, r, sinb, N, x2y2;
		int iter;
		iter = 0;
		r = 0.0;
		N = 0.0;
		sinb = 0.0;
		e2 = 2 * F - F * F;
		x2y2 = xyz->X * xyz->X + xyz->Y * xyz->Y;
		dZ = e2 * xyz->Z;
		do
		{
			Z = dZ;
			ZdZ = Z + xyz->Z;
			r = x2y2 + ZdZ*ZdZ;
			sinb = ZdZ / sqrt(r);
			N = a / sqrt(1 - e2*sinb*sinb);
			dZ = N * e2 * sinb;
			iter = iter + 1;
		} while ((iter <= 10) && (fabs(dZ - Z) > 1E-8));
		out->L = atan2(xyz->Y, xyz->X);
		out->B = atan2(ZdZ, sqrt(x2y2));
		out->H = sqrt(x2y2 + ZdZ*ZdZ) - N;
	}
	double distance_towards(XYZ * obj)
	{
		double temp1 = obj->X - X;
		double temp2 = obj->Y - Y;
		double temp3 = obj->Z - Z;
		return sqrt(temp1*temp1 + temp2*temp2 + temp3*temp3);
	}
};


class SpaceTool{
public:
	static double get_arc(double deg)
	{
		return deg * 180.0 / PI;
	}
	static double get_atan(double z, double y)
	{
		double x = 0;
		if (z == 0)x = PI / 2;
		else if (y == 0)x = PI;
		else {
			x = atan(fabs(y / z));
			if ((y > 0) && (z < 0))x = PI - x;
			else if ((y < 0) && (z < 0))x = PI + x;
			else if ((y < 0) && (z > 0))x = 2 * PI - x;
		}
		return x;
	}

	static void elevation_and_azimuth(XYZ * satellite, XYZ * user_location, double out[2])
	{
		double dpos[3] = {0};
		XYZ ori{ 0,0,0 };
		dpos[0] = satellite->X - user_location->X;
		dpos[1] = satellite->Y - user_location->Y;
		dpos[2] = satellite->Z - user_location->Z;

		double user_distance_to_earth = user_location->distance_towards(&ori);
		double mod = sqrt(dpos[0] * dpos[0] + dpos[1] * dpos[1] + dpos[2] * dpos[2]);
		if (fabs(user_distance_to_earth * mod < 1.0)) {
			out[0] = PI_d_2;
		}
		else{
			double m = dpos[0] * user_location->X + dpos[1] * user_location->Y + dpos[2] * user_location->Z;
			double n = m / (mod * user_distance_to_earth);
			out[0] = PI_d_2 - acos(n);
		}

		BLH blh = {0, 0, 0};
		user_location->toBLH(&blh);
		double B = blh.B;
		double L = blh.L;
		double N = -sin(B) * cos(L) * dpos[0] - sin(B) * sin(L) * dpos[1] + cos(B) * dpos[2];
		double E = -sin(L) * dpos[0] + cos(L)* dpos[1];
		out[1] = atan2(E, N);
	}

};