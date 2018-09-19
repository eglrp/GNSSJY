#pragma once
#include "MatC.h"
#include "DataStore.h"

#include "RINEX2.h"
#include "ProblemDef.h"

#include "Troposphere.h"

enum SolverType
{
	SPP_STATIC,
	PPP,
	SPP_KINEMATIC,
	Unknown,
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
protected:

	FILE * debug_file;
	int prn_list[GNSS_SATELLITE_AMOUNT];

	TYPE_OF_RINEX_OBS obs_type;

	XYZ satellite_position[GNSS_SATELLITE_AMOUNT];
	double observation[GNSS_SATELLITE_AMOUNT];
	double obs_var[GNSS_SATELLITE_AMOUNT];
	double phase[GNSS_SATELLITE_AMOUNT];
	double elev[GNSS_SATELLITE_AMOUNT];
	

	int satellite_amount;
	XYZ sat_temp;
	double ea[2];

	double TEC;

	//SatStatus satellite_summery[GNSS_SATELLITE_AMOUNT];

	Matrix * Z;
	Matrix * H;
	Matrix * D;
	Matrix * X;
	Matrix * Sig;
	Matrix * V;

	SimpleTroposphereModel tro;

	double S  [GNSS_SATELLITE_AMOUNT];
	double DX0[GNSS_SATELLITE_AMOUNT];
	double DY0[GNSS_SATELLITE_AMOUNT];
	double DZ0[GNSS_SATELLITE_AMOUNT];

	virtual void print_intermidiate()
	{
		for (int i = 0; i < satellite_amount; i++)
		{
			printf("%.10lf\t%.10lf\t%.10lf\t:\t%.10lf\n", satellite_position[i].X, satellite_position[i].Y, satellite_position[i].Z, observation[i]);
		}
		printf("S  DX0  DY0  DZ0:\n");
		for (int i = 0; i < satellite_amount; i++)
		{
			printf("%.10lf\t%.10lf\t%.10lf\t%.10lf\n", S[i], DX0[i], DY0[i], DZ0[i]);
		}
		
		mat_output(Z,     "Z");
		mat_output(H,     "H");
		mat_output(D,     "D");
		mat_output(X,     "X");
		mat_output(Sig, "Sig");
		mat_output(V,     "V");
	}
	virtual bool pre_process(GPSTime * pre, GNSSDataSet & set, int index, XYZ * sat_loc)
	{
		Observation & obs = set.obs[index];
		Broadcast  & nav = set.nav[index];
		SatelliteData & sat = set.sats[index];
		double dtc = pre->minus(&nav.toc) - obs.values[obs_type] / LIGHT_SPEED;
		double s =   nav.sv_clock_bias
				   + nav.sv_clock_drift      * dtc
				   + nav.sv_clock_drift_rate * dtc;

		double r = (R_1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);
		
		observation[satellite_amount] = obs.values[obs_type] + s * LIGHT_SPEED;//星钟差
		observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
		observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟
		phase[satellite_amount] = obs.values[L1] + s * LIGHT_SPEED / FREQ1;
		phase[satellite_amount] -= r * LIGHT_SPEED; //相对论
		phase[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟
		
		//地球自转改正
		double dA = we * (observation[satellite_amount] / LIGHT_SPEED - s + r);
		satellite_position[satellite_amount].X = sat_loc->X + sat_loc->Y * dA;
		satellite_position[satellite_amount].Y = sat_loc->Y - sat_loc->X * dA;
		satellite_position[satellite_amount].Z = sat_loc->Z;

		memcpy(&sat.time,                                       pre, sizeof(GPSTime));
		memcpy(&sat.position, &satellite_position[satellite_amount], sizeof(XYZ));

		/*if (set.solution_available())
		{
			tro.set_parameters(set.current_solution, UTC(*pre).get_doy());
			observation[satellite_amount] -= tro.get_overall_std(ea[0]);
		}*/
		// 电离层改正
		if(set.ion_model.good() && set.solution_available())
		{
			// under construction

		}
		else if(TEC != 0)
		{
			// under construction
		}

		satellite_amount ++;
		return true;
	}


	virtual bool get_static_solution(XYZ * current_solution, double & To)
	{
		XYZ before = {0, 0, 0};
		memcpy(&before, current_solution, sizeof(XYZ));

		free_mat(Z);
		free_mat(H);
		free_mat(D);
		free_mat(X);
		free_mat(Sig);
		free_mat(V);

		Z   = malloc_mat(satellite_amount, 1);
		H   = malloc_mat(satellite_amount, 4);
		D   = malloc_mat(satellite_amount, satellite_amount);
		V   = malloc_mat(satellite_amount, 1);
		Sig = malloc_mat(4, 4);
		X   = malloc_mat(4, 1);

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
				D->data[j][j] = obs_var[j];
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
				// job done
				//print_intermidiate();
				break;
			}

			memcpy(&before, current_solution, sizeof(XYZ));
		}



		return true;

	}

	virtual void fetch_available(GNSSDataSet & set, GPSTime * pre)
	{
		// tec ion
		if (set.solution_available() && set.tec.good(pre)) {
			BLH location;
			set.current_solution.toBLH(&location);
			// primary solution : find the nearest point.
			int col = (int)round((SpaceTool::get_deg(location.L) - set.tec.col_start) / set.tec.col_step);
			int row = (int)round((SpaceTool::get_deg(location.B) - set.tec.row_start) / set.tec.row_step);

			TEC = set.tec.data->data[row][col];
		}
		else TEC = 0;

		satellite_amount = 0;
		for(int i = 1; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			if (i == 8)
			{
				int j = 0;
			}
			if(set.nav[i].good() && set.obs[i].good(obs_type))
			{
				if(set.nav[i].get_position(
					pre,
					set.obs[i].values[obs_type],
					&sat_temp
					)
				)
				{
					if(set.solution_available())
					{
						SpaceTool::elevation_and_azimuth(&sat_temp, &set.current_solution, ea);
						if(SpaceTool::get_deg(ea[0]) < 10)
							continue;
						obs_var[satellite_amount] = 1.0 / sin(ea[0]);
					}
					else obs_var[satellite_amount] = 5;
					elev[satellite_amount] = SpaceTool::get_deg(ea[0]);
					prn_list[satellite_amount] = i;
					

					// make the typical corrections done
					if(!pre_process(pre, set, i, &sat_temp))
					{
						continue;
					}
					
				}
			}
		}
	}

public:
	SimpleSolver(TYPE_OF_RINEX_OBS obs = C1)
	{
		type = SolverType::SPP_STATIC;
		obs_type = obs;
		memset(satellite_position, 0, sizeof(XYZ)    * GNSS_SATELLITE_AMOUNT);
		memset(observation,        0, sizeof(double) * GNSS_SATELLITE_AMOUNT);
		sat_temp = { 0, 0 }; ea[0] = ea[1] = 0.0;
		satellite_amount = 0;
		TEC = 0;
		debug_file = fopen("debug.txt", "w");

		Z = NULL;
		H = NULL;
		D = NULL;
		X = NULL;
		Sig = NULL;
		V = NULL;
	}
	int get_satellite_amount()
	{
		return satellite_amount;
	}

	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);

		// firstly, calculate the positions of all available satellites
		fetch_available(set, pre);

		for (int i = 0; i < satellite_amount; i++)
		{
			if (elev[i] <= 10)continue;
			fprintf(debug_file, "%.1lf\t%2d\t%14.3lf\t%14.3lf\t%14.3lf\t%14.3lf\t%14.3lf\t%2.3lf\n",
				(double)pre->sec,
				prn_list[i],
				satellite_position[i].X,
				satellite_position[i].Y,
				satellite_position[i].Z,
				observation[i],
				phase[i],
				elev[i]
			);

		}
		if(satellite_amount <= 3) return false;

		get_static_solution(&set.current_solution, set.To);

		return true;
	}
};


class SimpleKinematicSolver : public SimpleSolver{
private:
	double DeltaT;
	double const_var_z = 0.01;
	double const_var_p = 1;
protected:
	//Matrix * X;//X Vx Ax Y Vy Ay Z Vz Az T dT
	Matrix * Xp;
	//Matrix * Dx; D
	Matrix * Dp;
	Matrix * F;
	Matrix * T;

	Matrix * Ft;
	Matrix * Tt;
	Matrix * De;

	//Matrix * Z;
	//Matrix * H;
	Matrix * Dz;
	Matrix * Zp;

	Matrix * K;

	//Matrix * V;

	void EKF_Execute()
	{
		//增益矩阵
		Matrix * temp1 = NULL, *temp2 = NULL, *temp3 = NULL, *temp4 = NULL;
		Matrix * Ht = NULL;
		mat_trans(H, Ht);
		mat_multiply(Dp, Ht, temp1);
		mat_multiply(H, Dp, temp2);
		mat_multiply(temp2, Ht, temp3);
		mat_sum(temp3, Dz);
		mat_inv(temp3, temp4);
		mat_multiply(temp1, temp4, K);
		free_mat(temp1); free_mat(temp2); free_mat(temp3); free_mat(temp4);

		//新息序列
		mat_minus(Z, Zp, V);
		
		//状态滤波
		mat_multiply(K, V, temp1);
		mat_sum(Xp, temp1, X);

		//滤波方差
		mat_multiply(K, H, temp2);
		Matrix * I = eyes(11);
		mat_minus(I, temp2, temp3);
		mat_multiply(temp3, Dp, D);

		free_mat(temp1); free_mat(temp2); free_mat(temp3);
	}

	void EKF_Predict()
	{
		mat_multiply(F, X, Xp);
		Matrix * temp1 = NULL, *temp2 = NULL, *temp3 = NULL, *temp4 = NULL;
		mat_multiply(F, D, temp1);
		mat_multiply(temp1, Ft, temp2);
		mat_multiply(T, De, temp3);
		mat_multiply(temp3, Tt, temp4);
		mat_sum(temp2, temp4, Dp);

		free_mat(temp1);
		free_mat(temp2);
		free_mat(temp3);
		free_mat(temp4);
	}
	void EKF_Reset()
	{
		free_mat(V);
		free_mat(K);
	}
	void EKF_FetchObservation(XYZ * current_solution, double & To)
	{
		free_mat(H);
		free_mat(Z);
		free_mat(Dz);
		free_mat(Zp);

		H = malloc_mat(satellite_amount, 11);
		Z = malloc_mat(satellite_amount, 1);
		Dz = malloc_mat(satellite_amount, satellite_amount);
		Zp = malloc_mat(satellite_amount, 1);

		XYZ predicted{ Xp->data[0][0], Xp->data[3][0], Xp->data[6][0] };

		for (int i = 0; i < satellite_amount; i++)
		{
			S[i] = predicted.distance_towards(&satellite_position[i]);

			DX0[i] = satellite_position[i].X - predicted.X;
			DY0[i] = satellite_position[i].Y - predicted.Y;
			DZ0[i] = satellite_position[i].Z - predicted.Z;
		}

		for (int i = 0; i < satellite_amount; i++)
		{
			Z->data[i][0] = observation[i];
			Dz->data[i][i] = const_var_z;

			H->data[i][0] = -DX0[i] / S[i];
			H->data[i][3] = -DY0[i] / S[i];
			H->data[i][6] = -DZ0[i] / S[i];
			H->data[i][9] = 1;

			Zp->data[i][0] = S[i] + Xp->data[9][0];
		}

		int i = 0;
	}

	//virtual void print_intermidiate()
	//{
	//	printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
	//		X->data[0][0],
	//		X->data[1][0],
	//		X->data[2][0],
	//		X->data[3][0],
	//		X->data[4][0],
	//		X->data[5][0],
	//		X->data[6][0],
	//		X->data[7][0],
	//		X->data[8][0],
	//		X->data[9][0],
	//		X->data[10][0]
	//	);
	//}

	bool get_kinematic_solution(XYZ * current_solution, double & To)
	{
		EKF_Predict();
		//mat_output(D, "D");
		//mat_output(X, "X");
		EKF_FetchObservation(current_solution, To);
		//mat_output(Z, "Z");
		//mat_output(Zp, "Zp");
		EKF_Execute();
		//mat_output(K, "K");
		//mat_output(V, "V");
		current_solution->X = X->data[0][0];
		current_solution->Y = X->data[3][0];
		current_solution->Z = X->data[6][0];
		To                  = X->data[9][0];

		EKF_Reset();

		return true;
	}

public:
	SimpleKinematicSolver(double sample_interval, TYPE_OF_RINEX_OBS obs = C1): SimpleSolver(obs)
	{
		type = SolverType::SPP_KINEMATIC;
		DeltaT = sample_interval;
		double ttd2 = 0.5 * DeltaT * DeltaT;
		double tttd6 = DeltaT * DeltaT * DeltaT / 6;
		
		Xp = malloc_mat(11, 1);
		
		Dp = eyes(11);

		Z = NULL; Dz = NULL; H = NULL; Ft = NULL; Tt = NULL; K = NULL; V = NULL; Zp = NULL;

		//Fai矩阵
		F = malloc_mat(11, 11);
		for (int i = 0; i < 3; i++)
		{
			int offset = i * 3;
			F->data[offset][offset] = 1;
			F->data[offset][offset + 1] = DeltaT;
			F->data[offset][offset + 2] = ttd2;
			F->data[offset + 1][offset + 1] = 1;
			F->data[offset + 1][offset + 2] = DeltaT;
			F->data[offset + 2][offset + 2] = 1;
		}
		F->data[9][9] = 1; F->data[9][10] = DeltaT; F->data[10][10] = 1;

		//Tao矩阵
		T = malloc_mat(11, 4);
		for (int i = 0; i < 3; i++)
		{
			T->data[3 * i][i] = tttd6;
			T->data[3 * i + 1][i] = ttd2;
			T->data[3 * i + 2][i] = DeltaT;
		}

		T->data[9][3] = ttd2; T->data[10][3] = DeltaT;

		De = eyes(4);
		De->data[0][0] = const_var_p; 
		De->data[1][1] = const_var_p; 
		De->data[2][2] = const_var_p; 
		De->data[3][3] = const_var_p;

		mat_trans(F, Ft); mat_trans(T, Tt);
	}


	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);

		// firstly, calculate the positions of all available satellites
		fetch_available(set, pre);

		if (!set.solution_available()) // no current solution found
		{
			if (satellite_amount <= 3) 
				return false; // see if satellite enough for static
			else {
				get_static_solution(&set.current_solution, set.To);
				X = malloc_mat(11, 1);
				D = eyes(11);
				X->data[0][0] = set.current_solution.X;
				X->data[3][0] = set.current_solution.Y;
				X->data[6][0] = set.current_solution.Z;
				X->data[9][0] = set.To;
				free_mat(V);
			}
		}
		else
			get_kinematic_solution(&set.current_solution, set.To);
		
		return true;
	}
};

class SimpleSmoothedSolver : public SimpleSolver{
private:
	TYPE_OF_RINEX_OBS phase;
	double lamda;
	double interval;
	GPSTime last_time;
	double last_observation[GNSS_SATELLITE_AMOUNT];
	double last_phase[GNSS_SATELLITE_AMOUNT];

	int period_counter;

	bool smoothing;
	double f1;
	double f2;
	double f3;
	double delta;
protected:
	virtual void fetch_available(GNSSDataSet & set, GPSTime * pre)
	{

		// tec ion
		if (set.solution_available() && set.tec.good(pre)) {
			BLH location;
			set.current_solution.toBLH(&location);
			// primary solution : find the nearest point.
			int col = (int)round((SpaceTool::get_deg(location.L) - set.tec.col_start) / set.tec.col_step);
			int row = (int)round((SpaceTool::get_deg(location.B) - set.tec.row_start) / set.tec.row_step);

			TEC = set.tec.data->data[row][col];
		}
		else TEC = 0;

		satellite_amount = 0;
		for (int i = 1; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			if (i == 8)
			{
				int j = 0;
			}
			if (set.nav[i].good() && set.obs[i].good(obs_type))
			{
				if (set.nav[i].get_position(
					pre,
					set.obs[i].values[obs_type],
					&sat_temp
				)
					)
				{
					if (set.solution_available())
					{
						SpaceTool::elevation_and_azimuth(&sat_temp, &set.current_solution, ea);
						if (SpaceTool::get_deg(ea[0]) < 10)
							continue;
						obs_var[satellite_amount] = 1.0 / sin(ea[0]);
					}
					else obs_var[satellite_amount] = 5;
					elev[satellite_amount] = SpaceTool::get_deg(ea[0]);
					prn_list[satellite_amount] = i;


					// make the typical corrections done
					if (!pre_process(pre, set, i, &sat_temp))
					{
						continue;
					}

				}
			}
		}
	}

	virtual bool pre_process(GPSTime * pre, GNSSDataSet & set, int index, XYZ * sat_loc)
	{
		Observation & obs = set.obs[index];
		Broadcast  & nav = set.nav[index];		

		if(smoothing)
		{
			if (!obs.good(phase))return false;

			delta = lamda * (obs.values[phase] - last_phase[index]);
			f1 = obs.values[obs_type] / period_counter;
			f2 = last_observation[index] + delta;
			f3 = (period_counter - 1.0) / period_counter * f2;

			obs.values[obs_type] = f1 + f3;
		}
		
		last_observation[index] = obs.values[obs_type];
		last_phase[index] = obs.values[phase];

		double dtc = pre->minus(&nav.toc) - obs.values[obs_type] / LIGHT_SPEED;
		double s =   nav.sv_clock_bias
				   + nav.sv_clock_drift      * dtc
				   + nav.sv_clock_drift_rate * dtc;

		double r = (R_1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);
		observation[satellite_amount] = obs.values[obs_type] + s * LIGHT_SPEED;//星钟差
		observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
		observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟
		
		//地球自转改正
		double dA = we * (observation[satellite_amount] / LIGHT_SPEED - s + r);
		satellite_position[satellite_amount].X = sat_loc->X + sat_loc->Y * dA;
		satellite_position[satellite_amount].Y = sat_loc->Y - sat_loc->X * dA;
		satellite_position[satellite_amount].Z = sat_loc->Z;

		satellite_amount ++;
		return true;
	}
public:

	SimpleSmoothedSolver(double sample_interval, TYPE_OF_RINEX_OBS code = C1): SimpleSolver(code)
	{
		switch(code)
		{
			case C1:
				phase = L1;
				lamda = LIGHT_SPEED / FREQ1; // L1波长
				break;
			case C2:
				phase = L2;
				lamda = LIGHT_SPEED / FREQ2; // L2波长
				break;
			default:
				throw INVALID_INPUT;
		}
		memset(last_observation, 0, sizeof(double) * GNSS_SATELLITE_AMOUNT);
		memset(last_phase, 0, sizeof(double) * GNSS_SATELLITE_AMOUNT);
		last_time = GPSTime(0, 0);
		interval = sample_interval;
		smoothing = true;
		period_counter = 0;
	}

	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);

		smoothing = (pre->minus(&last_time) == interval);
		period_counter = (pre->minus(&last_time) == interval) ? period_counter + 1 : 1;
		
		// calculate the positions of all available satellites
		fetch_available(set, pre);
		if (satellite_amount <= 3)
			return false;

		get_static_solution(&set.current_solution, set.To);

		memcpy(&last_time, pre, sizeof(GPSTime));
		return true;
	}
};


class SequencedSimpleSolver : public SimpleSolver
{
protected:

	//Matrix * Qpinv;
	Matrix * K;
	Matrix * Q;
	Matrix * W;
	Matrix * h;
	Matrix * w;
	Matrix * z;
	Matrix * x;

public:
	SequencedSimpleSolver() : SimpleSolver()
	{
		W = eyes(satellite_amount);
		H = malloc_mat(satellite_amount, 4);
		X = malloc_mat(4, 1);
	}

	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);
		fetch_available(set, pre);
		if (satellite_amount <= 0)
		{
			return false;	
		}
		get_sequenced_solution(&set.current_solution, set.To);
		return true;
	}

	virtual bool get_sequenced_solution(XYZ * current_solution, double & To)
	{
		XYZ before = { 0, 0, 0 };

		z   = malloc_mat(satellite_amount,                1);
		h   = malloc_mat(satellite_amount,                4);
		w   = malloc_mat(satellite_amount, satellite_amount);
		x   = malloc_mat(               4,                1);

		for (int j = 0; j < satellite_amount; j++)
		{
			S[j] = current_solution->distance_towards(&satellite_position[j]);
			DX0[j] = satellite_position[j].X - current_solution->X;
			DY0[j] = satellite_position[j].Y - current_solution->Y;
			DZ0[j] = satellite_position[j].Z - current_solution->Z;
		}

		for (int j = 0; j < satellite_amount; j++)
		{
			w->data[j][j] = 1;
			z->data[j][0] = observation[j] - S[j] - To;
			h->data[j][0] = -DX0[j] / S[j];
			h->data[j][1] = -DY0[j] / S[j];
			h->data[j][2] = -DZ0[j] / S[j];
			h->data[j][3] = 1;
		}
		Matrix * temp1 = NULL, *temp2 = NULL, *Ht = NULL, *ht = NULL, *temp3 = NULL;
		Matrix * dz = NULL, *Qinv = NULL;

		//mat_trans(H, Ht);
		//mat_multiply(Ht, W, temp1);
		//mat_multiply(temp1, H, temp2);  // HWH
		mat_trans(h, ht);
		mat_multiply(ht, w, temp1);

		mat_multiply(temp1, h, temp3);  // hwh

		mat_addition(temp2, temp3, Qinv);

		mat_inv(temp3, Q);

		mat_multiply(Q, ht, temp1);
		mat_multiply(temp1, w, K); // Qhw
		mat_multiply(h, X, temp1); // hX
		mat_minus(z, temp1, dz); // z - hX
		mat_multiply(K, dz, temp1);
		mat_addition(temp1, X, x);



		free_mat(dz);
		free_mat(temp1);
		free_mat(temp2);
		free_mat(temp3);
		free_mat(ht);
		free_mat(Ht);
		//mat_output(Qpinv, "Qpinv");

		current_solution->X += x->data[0][0];
		current_solution->Y += x->data[1][0];
		current_solution->Z += x->data[2][0];
		To += x->data[3][0];


		if (before.distance_towards(current_solution) <= 0.001) {

			// so what
		}

		memcpy(&before, current_solution, sizeof(XYZ));

		//free_mat(Qpinv);
		//Qpinv = Qinv;

		free_mat( W );
		free_mat( K );
		free_mat( Q );
		
		free_mat( Z );
		free_mat( H );
		free_mat( X );
		H = h;
		W = w;
		X = x;

		return true;
	}
};

class SimplePPPSolver : public Solver
{
protected:
	//some const values
	const double if_n = 2.545727780163160;
	const double if_m = -1.545727780163160;
	const double lam_if = 0.190293672798365;
	const double lam_1 = 0.190293672798365;
	const double lam_2 = 0.244210213424568;

#define OBS_NUM 4
	TYPE_OF_RINEX_OBS obs_types[OBS_NUM]{ C1, P2, L1, L2 };

	// interface of sp solver;
	SimpleSolver spp_solver;
	// interface of the troposphere model;
	SimpleTroposphereModel tro_model;


	// matrices for calculating
	Matrix * L; // for Z observation matrix 
	Matrix * A; // for H design(systematic) matrix
	Matrix * W; // for weight matrix
	Matrix * Q; // 
	Matrix * J; // for Gain matrix
	Matrix * x; // for current unknown vector

	// previous matrices
	Matrix * Wp; // for previous weight matrix
	Matrix * Ap;// 
	Matrix * X; // for previous unknown vector
	//Matrix * Qp; //

	// some passing-on paramters
	double obs_p   [GNSS_SATELLITE_AMOUNT];
	double obs_l   [GNSS_SATELLITE_AMOUNT];
	XYZ    sat_locs[GNSS_SATELLITE_AMOUNT];
	int    prn_list[GNSS_SATELLITE_AMOUNT];
	int    last_prn[GNSS_SATELLITE_AMOUNT];
	double tro_m   [GNSS_SATELLITE_AMOUNT];
	double d_N     [GNSS_SATELLITE_AMOUNT];
	int    satellite_amount;
	int    last_sat_amount;

	XYZ last_solution;
	double last_t0;
	
	// temp usage
	double ea[2]; // elevation_and_azimuth

	double DX[GNSS_SATELLITE_AMOUNT];
	double DY[GNSS_SATELLITE_AMOUNT];
	double DZ[GNSS_SATELLITE_AMOUNT];
	double  S[GNSS_SATELLITE_AMOUNT];

	// programming flags
	bool is_changed;
	bool is_first;

	bool get_ppp_solution(GNSSDataSet & set)
	{
		A = malloc_mat(satellite_amount * 2, satellite_amount + 5);
		W = malloc_mat(satellite_amount * 2, satellite_amount * 2);
		L = malloc_mat(satellite_amount * 2, 1);
		X = malloc_mat(satellite_amount + 5, 1);

		//for (int i = 0; i < 20; i++) 
		{
			// get X
			for (int i = 0; i < 5; i++)
				X->data[i][0] = x->data[i][0]; // for xyz dt //trop
			//X->data[0][0] = last_solution.X - set.current_solution.X + x->data[0][0];
			//X->data[1][0] = last_solution.Y - set.current_solution.Y + x->data[1][0];
			//X->data[2][0] = last_solution.Z - set.current_solution.Z + x->data[2][0];
			//X->data[3][0] = last_t0 - set.To + x->data[3][0];
			free_mat(x);

			for (int i = 0; i < satellite_amount; i++)
				X->data[5 + i][0] = d_N[prn_list[i]]; // for Ns


			//mat_output(X, "X");
		
			for (int i = 0; i < satellite_amount; i++)
			{
				S[i] = set.current_solution.distance_towards(sat_locs + i);
				DX[i] = sat_locs[i].X - set.current_solution.X;
				DY[i] = sat_locs[i].Y - set.current_solution.Y;
				DZ[i] = sat_locs[i].Z - set.current_solution.Z;
			}

			for (int i = 0; i < satellite_amount; i++)
			{
				// 0 for psedorange obs, 1 for phase obs
				L->data[i * 2 + 0][0] = obs_p[i] - S[i] - set.To - set.dtrop * tro_m[i];
				L->data[i * 2 + 1][0] = obs_l[i] - S[i] - set.To - set.dtrop * tro_m[i] + set.sats[prn_list[i]].ambiguity * lam_if;
				W->data[i * 2 + 0][i * 2 + 0] = 0.0887;
				W->data[i * 2 + 1][i * 2 + 1] = 8.7813;

				for (int j = 0; j < 2; j++) {
					A->data[i * 2 + j][0] = -DX[i] / S[i]; // X
					A->data[i * 2 + j][1] = -DY[i] / S[i]; // Y
					A->data[i * 2 + j][2] = -DZ[i] / S[i]; // Z
					A->data[i * 2 + j][3] = 1;             // dt
					A->data[i * 2 + j][4] = tro_m[i];
				}

				A->data[i * 2 + 1][i + 5] = 1;             // N
			}

			if (is_changed || is_first)
			{
				free_mat(Ap);
				free_mat(Wp);
				//mat_clear(Ap);
				//mat_clear(Wp);
				Ap = malloc_mat(satellite_amount * 2, satellite_amount + 5);//copy_mat(A);
				Wp = malloc_mat(satellite_amount * 2, satellite_amount * 2);//copy_mat(W);
			}

			Matrix * temp1 = NULL, *temp2 = NULL, *Apt = NULL, *At = NULL, *temp3 = NULL;
			Matrix * dz = NULL, *Qinv = NULL;

			mat_trans(Ap, Apt);
			mat_multiply(Apt, Wp, temp1);
			mat_multiply(temp1, Ap, temp2);  // HWH
			mat_trans(A, At);
			mat_multiply(At, W, temp1);

			mat_multiply(temp1, A, temp3);  // hwh

			mat_addition(temp2, temp3, Qinv);
			mat_inv(temp3, Q);
			mat_multiply(Q, At, temp1);
			mat_multiply(temp1, W, J); // Qhw
			mat_multiply(A, X, temp1); // hX
			mat_minus(L, temp1, dz); // z - hX
			mat_multiply(J, dz, temp1);
			mat_addition(temp1, X, x);



			last_solution = set.current_solution;
			last_t0 = set.To;


			set.current_solution.X += x->data[0][0];
			set.current_solution.Y += x->data[1][0];
			set.current_solution.Z += x->data[2][0];
			set.To += x->data[3][0];
			set.dtrop += x->data[4][0];


			for (int i = 0; i < satellite_amount; i++) {
				set.sats[prn_list[i]].ambiguity = x->data[5 + i][0];
				d_N[prn_list[i]] = x->data[5 + i][0];
			}
			free_mat(dz);
			free_mat(temp1);
			free_mat(temp2);
			free_mat(temp3);
			free_mat(At);
			free_mat(Apt);
		}

		free_mat(Wp);
		free_mat(J);
		free_mat(Q);

		free_mat(L);
		free_mat(Ap);
		free_mat(X);
		Ap = A;
		Wp = W;


		return true;
	}

	void fetch_available(GNSSDataSet & set, GPSTime * pre)
	{
		last_sat_amount = satellite_amount;
		memcpy(last_prn, prn_list, sizeof(double) * satellite_amount);
		satellite_amount = 0;
		for (int i = 1; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			if (i == 30)
			{
				int j = 0;
			}
			if (set.obs[i].good(obs_types, OBS_NUM))
			{
				if (set.sats[i].available(pre))
				{
					SpaceTool::elevation_and_azimuth(&set.sats[i].position, &set.current_solution, ea);
					if (SpaceTool::get_deg(ea[0]) < 10)
						continue;

					Broadcast   & nav = set.nav[i];
					Observation & obs = set.obs[i];
					//obs_p[satellite_amount] = obs.values[C1];
					//obs_l[satellite_amount] = obs.values[L1];
					obs_p[satellite_amount] = obs.values[C1] * if_n + obs.values[P2] * if_m;
					obs_l[satellite_amount] = obs.values[L1] * lam_1 * if_n + obs.values[L2] * lam_2 * if_m;
					double dtc = pre->minus(&nav.toc) - obs.values[C1] / LIGHT_SPEED;
					double s = nav.sv_clock_bias
						+ nav.sv_clock_drift      * dtc
						+ nav.sv_clock_drift_rate * dtc;

					double r = (R_1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);

					obs_p[satellite_amount] -= tro_model.get_dry_std(ea[0]); // 对流层干分量
					obs_p[satellite_amount] += s       *        LIGHT_SPEED; // 星钟差
					obs_p[satellite_amount] -= r       *        LIGHT_SPEED; // 相对论
					obs_p[satellite_amount] -= nav.tgd *        LIGHT_SPEED; // 群延迟

					obs_l[satellite_amount] -= tro_model.get_dry_std(ea[0]); // 对流层干分量
					obs_l[satellite_amount] += s       *        LIGHT_SPEED; // 星钟差
					obs_l[satellite_amount] -= r       *        LIGHT_SPEED; // 相对论
					obs_l[satellite_amount] -= nav.tgd *        LIGHT_SPEED; // 群延迟

					if(is_first)
						tro_m[satellite_amount] = tro_model.get_wet_m(ea[0]);    // 对流层湿分量投影函数

					sat_locs[satellite_amount] = set.sats[i].position;
					prn_list[satellite_amount] = i;

					satellite_amount++;
				}
			}
		}
		if (is_first)
			is_changed = false;
		else if (last_sat_amount != satellite_amount)
			is_changed = true;
		else if (memcmp(last_prn, prn_list, sizeof(double) * satellite_amount) != 0)
			is_changed = true;
		else 
			is_changed = false;
	}

public:
	SimplePPPSolver(): Solver()
	{
		//spp_solver = SimpleSolver();
		L = NULL;
		A = NULL;
		Q = NULL;
		is_first = true;
		is_changed = false;
		memset(d_N, 0, sizeof(double) * GNSS_SATELLITE_AMOUNT);
	}

	bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);

		if (set.obs_time.minute == 23)
		{
			int i = 0;
		}
		if (pre->sec == 437850)
		{
			int i = 0;
		}
		// for SPP

		if (!spp_solver.execute(set))
			return false;

		if (is_first) {
			tro_model.set_parameters(set.current_solution, set.obs_time.get_doy());
			fetch_available(set, pre);
			set.dtrop = tro_model.get_wet_ztd();
			is_first = false;
			last_solution = set.current_solution;
			last_t0 = set.To;
			// matrices alloc
			x = malloc_mat(satellite_amount + 5, 1);
			Wp = eyes(satellite_amount * 2);
			Ap = malloc_mat(satellite_amount * 2, satellite_amount + 5);
			//Qpinv = eyes(satellite_amount + 4);
			
		}
		else
		{
			//set.current_solution = last_solution;
			fetch_available(set, pre);
		}

		get_ppp_solution(set);
		//last_solution = set.current_solution;
		//last_t0 = set.To;
		return true;
	}
	
};