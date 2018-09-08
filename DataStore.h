#pragma once
#include <stdlib.h>
#include <string.h>
#include "Space.h"
#include "JTime.h"
//#include "RINEX2.h"
#define MAX_OBSER_TYPE 9
#define GNSS_SATELLITE_AMOUNT 50


#define LIGHT_SPEED 299792458.0
#define mu 3.986004415E14
#define we 7.2921151467e-5
#define R_1 4.442807633e-10
#define SECPERDAY (3600*24)
#define PI2 (PI*2)
#define Deg (180.0/PI)

#define OBS_DISABLED -888


enum TYPE_OF_RINEX_OBS {
	L1 = 0, L2 = 1, P1 = 2, P2 = 3, C1 = 4, C2 = 5, S1 = 6, S2 = 7, D1 = 8
};

struct StationInfo{
	XYZ approx_position;
	ENU antenna_delta;
};

struct checkable{
	
	bool good()
	{
		return available;
	}
	void check(bool status)
	{
		available = status;
	}
protected:
	bool available;
};


struct Broadcast: public checkable{ // using RINEX definition for common usage.

	// PRN / EPOCH / SV CLK
	GPSTime toc;
	double sv_clock_bias;
	double sv_clock_drift;
	double sv_clock_drift_rate;

	// BROADCAST ORBIT - 1
	double idoe_issue_of_data;
	double crs;
	double delta_n;
	double m0;

	// BROADCAST ORBIT - 2
	double cuc;
	double eccentricity;
	double cus;
	double sqrt_a;

	// BROADCAST ORBIT - 3
	double toe;
	double cic;
	double OMEGA;
	double cis;

	// BROADCAST ORBIT - 4
	double i0;
	double crc;
	double omega;
	double omega_dot;

	// BROADCAST ORBIT - 5
	double idot;
	double codes_on_l2;
	double gpsweek;
	double l2_pdata_flag;

	// BROADCAST ORBIT - 6
	double sv_accuracy;
	double sv_health;
	double tgd;
	double iodc_issue_of_data;

	// BROADCAST ORBIT - 7
	double trans_time;
	double fit_interval;

	// you can call this parameter only after you called get_position
	double Ek;
	bool get_position(GPSTime * pre, double approx_distance, XYZ * out)
	{
		//平均角速度n
		double n0 = sqrt(mu) / pow(sqrt_a, 3);
		double n = n0 + delta_n;

		//归化时间tk
		//GPSTime pre_gps = GPSTime(pre);
		double t = pre->sec - approx_distance / LIGHT_SPEED;
		double tk = t - toe;
		if (fabs(tk)>7200)
		{
			return false;
		}
		if (tk > 302400)
			tk -= 604800;
		else if (tk < -302400)
			tk += 604800;

		//平近点角Mk
		double Mk = m0 + n * tk;

		//偏近点角Ek
		Ek = Mk;
		double Ek2 = 0;
		while (1)
		{
			Ek2 = Mk + eccentricity * sin(Ek);
			if (fabs(Ek - Ek2) <= 1.0e-12)break;
			Ek = Ek2;
		}

		//真近点角fk
		double sqt_1_e2 = sqrt(1 - pow(eccentricity, 2));
		double cosfk = (cos(Ek) - eccentricity) / (1 - eccentricity * cos(Ek));
		double sinfk = (sqt_1_e2 * sin(Ek)) / (1 - eccentricity * cos(Ek));
		double fk = SpaceTool::get_atan((cos(Ek) - eccentricity), sqt_1_e2 * sin(Ek));
		//升交距角faik
		double faik = fk + omega;

		//摄动改正项su、sr、si
		double cos_2_faik = cos(2 * faik);
		double sin_2_faik = sin(2 * faik);
		double su = cuc * cos_2_faik + cus * sin_2_faik;
		double sr = crc * cos_2_faik + crs * sin_2_faik;
		double si = cic * cos_2_faik + cis * sin_2_faik;

		//升交距角uk、卫星矢径rk、轨道倾角ik
		double uk = faik + su;
		double rk = sqrt_a * sqrt_a * (1 - eccentricity * cos(Ek)) + sr;
		double ik = i0 + si + idot * tk;

		//卫星在轨道平面上的位置
		double xk = rk * cos(uk);
		double yk = rk * sin(uk);
		double zk = 0;

		//计算观测时刻的升交点经度L
		double L = OMEGA + (omega_dot - we) * tk - we * toe;
		
		//计算卫星在WGS84下的位置
		out->X = xk * cos(L) - yk * cos(ik) * sin(L);
		out->Y = xk * sin(L) + yk * cos(ik) * cos(L);
		out->Z = yk * sin(ik);

		return true;
	}
};

struct Observation: public checkable{
	double values[MAX_OBSER_TYPE];
	unsigned char LLI[MAX_OBSER_TYPE];             // optional
	unsigned char signal_strength[MAX_OBSER_TYPE]; // optional

	bool good(TYPE_OF_RINEX_OBS type)
	{
		if (available)
			if (values[type] != OBS_DISABLED)
				return true;
		return false;
	}
};



struct GNSSDataSet{
	UTC obs_time;
	double To; // for reciever clock bias

	// inputs
	Broadcast nav[GNSS_SATELLITE_AMOUNT];
	Observation obs[GNSS_SATELLITE_AMOUNT];

	StationInfo * sta;

	// passing-on
	XYZ current_solution;
	GNSSDataSet()
	{
		memset(nav, 0, sizeof(Broadcast) * GNSS_SATELLITE_AMOUNT);
		memset(obs, 0, sizeof(Observation) * GNSS_SATELLITE_AMOUNT);

		current_solution = {0, 0, 0};
		To = 0;
	}
};