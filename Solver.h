#pragma once
#include "MatC.h"
#include "DataStore.h"
#include "RINEX2.h"


enum SolverType
{
	SPP,
	PPP,
	Unknown
};

enum SatStatus{
	NO_ANY,
	NO_NAV,
	NO_OBS,
	TOO_LOW,
	BAN_LIST,
	AVAIL,
	UNKNOWN,
};

class Solver{
protected:
	SolverType type;
	
	
public:
	//virtual wstring description() = 0; 
	virtual bool execute(GNSSDataSet & set) = 0;
};


// simple spp solver using code pseudoranges.
class SimpleSolver: public Solver{
private:
	XYZ satellite_position[GNSS_SATELLITE_AMOUNT];
	double observation[GNSS_SATELLITE_AMOUNT];

	int satellite_amount;
	XYZ sat_temp;
	double ea[2];

	//SatStatus satellite_summery[GNSS_SATELLITE_AMOUNT];

	Matrix * Z;
	Matrix * H;
	Matrix * D;
	Matrix * X;
	Matrix * Sig;
	Matrix * V;

	double S  [GNSS_SATELLITE_AMOUNT];
	double DX0[GNSS_SATELLITE_AMOUNT];
	double DY0[GNSS_SATELLITE_AMOUNT];
	double DZ0[GNSS_SATELLITE_AMOUNT];

	bool pre_process(GPSTime * pre, GNSSDataSet & set, int index, XYZ * sat_loc)
	{
		Observation & obs = set.obs[index];
		Broadcast  & nav = set.nav[index];
		double dtc = pre->minus(&nav.toc) - obs.values[C1] / c;
		double s =   nav.sv_clock_bias
				   + nav.sv_clock_drift      * dtc
				   + nav.sv_clock_drift_rate * dtc;

		double r = (R1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);
		observation[satellite_amount] = obs.values[C1] + s * c;//星钟差
		observation[satellite_amount] -= r * c; //相对论
		observation[satellite_amount] -= nav.tgd * c;//群延迟
		
		//地球自转改正
		double dA = we * (observation[satellite_amount] / c - s + r);
		satellite_position[satellite_amount].X = sat_loc->X + sat_loc->Y * dA;
		satellite_position[satellite_amount].Y = sat_loc->Y - sat_loc->X * dA;
		satellite_position[satellite_amount].Z = sat_loc->Z;

		satellite_amount ++;
		return true;
	}

	bool get_solution(XYZ * current_solution, double & To)
	{
		XYZ before = {0, 0, 0};
		memcpy(&before, current_solution, sizeof(XYZ));

		Z = malloc_mat(satellite_amount, 1);
		H = malloc_mat(satellite_amount, 4);
		D = malloc_mat(satellite_amount, satellite_amount);
		V = malloc_mat(satellite_amount, 1);
		Sig = malloc_mat(4, 4);
		X = malloc_mat(4, 1);

		for(int i = 0;i < 20; i++)
		{
			for(int j = 0; j < satellite_amount; j++)
			{
				S[j] = current_solution->distance_towards(&satellite_position[j]);
				DX0[j] = satellite_position[j].X - current_solution->X;
				DY0[j] = satellite_position[j].Y - current_solution->Y;
				DZ0[j] = satellite_position[j].Z - current_solution->Z;
			}

			for (int j = 0; j < satellite_amount; j++)
			{
				D->data[j][j] = 0.001;
				//设置观测矩阵Z,Zi = Pi - P0i,P0i = Dis(X0,Si)+To
				Z->data[j][0] = observation[j] - S[j] - To;
				//设置系数矩阵
				H->data[j][0] = -DX0[j] / S[j];
				H->data[j][1] = -DY0[j] / S[j];
				H->data[j][2] = -DZ0[j] / S[j];
				H->data[j][3] = 1;
			}

			// 天地大同
			LMS(Z, H, D, X, Sig, V);



			current_solution->X += X->data[0][0];
			current_solution->Y += X->data[1][0];
			current_solution->Z += X->data[2][0];
			To                  += X->data[3][0];

			if (before.distance_towards(current_solution) <= 0.001) {
				//for (int i = 0; i < satellite_amount; i++)
				//{
				//	printf("%.10lf\t%.10lf\t%.10lf\t:\t%.10lf\n", satellite_position[i].X, satellite_position[i].Y, satellite_position[i].Z, observation[i]);
				//}
				//printf("S  DX0  DY0  DZ0:\n");
				//for (int i = 0; i < satellite_amount; i++)
				//{
				//	printf("%.10lf\t%.10lf\t%.10lf\t%.10lf\n", S[i], DX0[i], DY0[i], DZ0[i]);
				//}
				//
				//mat_output(Z, "Z");
				//mat_output(H, "H");
				//mat_output(D, "D");
				//mat_output(X, "X");
				//mat_output(Sig, "Sig");
				//mat_output(V, "V");
				break;
			}

			memcpy(&before, current_solution, sizeof(XYZ));
		}

		free_mat(Z);
		free_mat(H);
		free_mat(D);
		free_mat(X);
		free_mat(Sig);
		free_mat(V);

		return true;

	}
public:
	SimpleSolver()//: type(SPP)
	{
		type = SolverType::SPP;
		memset(satellite_position, 0, sizeof(XYZ)    * GNSS_SATELLITE_AMOUNT);
		memset(observation,        0, sizeof(double) * GNSS_SATELLITE_AMOUNT);
		sat_temp = { 0, 0 }; ea[0] = ea[1] = 0.0;
		satellite_amount = 0;

	}

	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);
		satellite_amount = 0;

		// firstly, calculate the positions of all available satellites
		for(int i = 1; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			if(set.nav[i].good() && set.obs[i].good(C1))
			{
				if(set.nav[i].get_position(
					pre,
					set.obs[i].values[C1],
					&sat_temp
					)
				)
				{
					if(set.current_solution.X != 0)
					{
						SpaceTool::elevation_and_azimuth(&sat_temp, &set.current_solution, ea);
						if(ea[0] * 180.0 / PI < 10)
							continue;
					}

					// make the typical corrections done
					pre_process(pre, set, i, &sat_temp);
				}
			}
		}

		if(satellite_amount <= 3) return false;

		get_solution(&set.current_solution, set.To);
		return true;
	}

	// void set_station_info(StationInfo * sta) {
	//  	memcpy(&station_data, sta, sizeof(StationInfo));
	// }
	//virtual wstring description()
	//{
	//	// under construction
	//	return
	//}
};
