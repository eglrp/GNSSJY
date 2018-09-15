#pragma once
#include "Solver.h"

enum STAGE {
	INIT,
	SEQU,
};

class DIYSolver : public SimpleSolver {
	Matrix * T;

	STAGE stage;
	int PRN[GNSS_SATELLITE_AMOUNT];
	int LAST_PRN[GNSS_SATELLITE_AMOUNT];
	int last_satellite_amount;

	int init_epoch;
	int required_init_epoch;

	int init_obs_num;
	XYZ init_sat_loc[GNSS_SATELLITE_AMOUNT];
	double init_obs[GNSS_SATELLITE_AMOUNT];
	int init_num_of_param;

	double lambdaL1;

	double N[GNSS_SATELLITE_AMOUNT];

	FILE * fp;
#define NUM_OF_OBS 2
	TYPE_OF_RINEX_OBS used[NUM_OF_OBS] = { L1, C1 };

	void reset_satellite()
	{
		last_satellite_amount = satellite_amount;
		memcpy(LAST_PRN, PRN, sizeof(int) * GNSS_SATELLITE_AMOUNT);
	}

	bool satellite_status_changed()
	{
		if (satellite_amount != last_satellite_amount)
		{
			reset_satellite();
			return true;
		}
		for (int i = 0; i < satellite_amount; i++)
			if (PRN[i] != LAST_PRN[i])
			{
				reset_satellite();
				return true;
			}
		reset_satellite();
		return false;
	}

public:
	DIYSolver(): SimpleSolver() {
		stage = INIT;
		init_epoch = 0;
		required_init_epoch = 5;
		init_obs_num = 0;
		init_num_of_param = 0;
		lambdaL1 = LIGHT_SPEED / FREQ1;
		memset(N, 0, sizeof(double) * GNSS_SATELLITE_AMOUNT);
		fp = fopen("wamb1.txt", "w");
	}
	void print_amb()
	{
		for (int i = 0; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			fprintf(fp, "%7.5lf ", N[i]);
		}
		fprintf(fp, "\n");
	}
	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);

		fetch_available(set, pre);
		if (satellite_status_changed() || satellite_amount < 4) {
			stage = INIT;
			init_epoch = 0;
			init_obs_num = 0;
			return false;
		}

		if (stage == INIT)
		{
			memcpy(init_sat_loc + init_obs_num, satellite_position, sizeof(XYZ)    * satellite_amount);
			memcpy(init_obs     + init_obs_num,        observation, sizeof(double) * satellite_amount);
			init_obs_num += satellite_amount;
			init_epoch++;
			if (init_epoch == required_init_epoch)
			{
				init_num_of_param = satellite_amount + 3 + required_init_epoch;
				if (init_num_of_param > init_obs_num) throw UNEXPECTED;
				stage = SEQU;
				get_init_solution(&set.current_solution, set.To);
				print_amb();
				stage = INIT;
				init_epoch = 0;
				init_obs_num = 0;
				return true;
			}
			
			return false;
		}

		get_static_solution(&set.current_solution, set.To);

		return true;
	}

protected:
	bool get_sequence_solution(XYZ * current_solution, double & To)
	{
		XYZ before = *current_solution;

		Z = malloc_mat(init_obs_num, 1);
		H = malloc_mat(init_obs_num, init_num_of_param);
		D = malloc_mat(init_obs_num, init_obs_num);
		V = malloc_mat(init_obs_num, 1);
		Sig = malloc_mat(init_num_of_param, init_num_of_param);
		X = malloc_mat(init_num_of_param, 1);

		for (int i = 0; i < 20; i++)
		{
			for (int j = 0; j < init_obs_num; j++)
			{
				S[j] = current_solution->distance_towards(&init_sat_loc[j]);
				DX0[j] = init_sat_loc[j].X - current_solution->X;
				DY0[j] = init_sat_loc[j].Y - current_solution->Y;
				DZ0[j] = init_sat_loc[j].Z - current_solution->Z;
			}

			for (int j = 0; j < init_obs_num; j++)
			{
				D->data[j][j] = 0.01;
				Z->data[j][0] = init_obs[j] - S[j];
				H->data[j][0] = -DX0[j] / S[j];
				H->data[j][1] = -DY0[j] / S[j];
				H->data[j][2] = -DZ0[j] / S[j];
			}

			for (int j = 0; j < required_init_epoch; j++)
			{
				for (int k = 0; k < satellite_amount; k++)
				{
					H->data[j * satellite_amount + k][3 + k] = lambdaL1;
					H->data[j * satellite_amount + k][3 + satellite_amount + j] = -LIGHT_SPEED;
				}
			}

			LMS(Z, H, D, X, Sig, V);

			current_solution->X += X->data[0][0];
			current_solution->Y += X->data[1][0];
			current_solution->Z += X->data[2][0];
			for (int j = 0; j < required_init_epoch; j++)
			{
				T->data[j][0] += X->data[3 + satellite_amount + j][0];
			}
			for (int j = 0; j < satellite_amount; j++)
			{
				N[PRN[j]] += round(X->data[3 + j][0]);
			}

			if (before.distance_towards(current_solution) <= 0.00001) {

				break;
			}

			memcpy(&before, current_solution, sizeof(XYZ));
		}


		free_mat(T);
		free_mat(Z);
		free_mat(H);
		free_mat(D);
		free_mat(X);
		free_mat(Sig);
		free_mat(V);



		return true;
	}

	bool get_init_solution(XYZ * current_solution, double & To)
	{
		XYZ before = { 0 , 0 , 0 };

		T   = malloc_mat(required_init_epoch, 1);
		Z   = malloc_mat(init_obs_num, 1);
		H   = malloc_mat(init_obs_num, init_num_of_param);
		D   = malloc_mat(init_obs_num, init_obs_num);
		V   = malloc_mat(init_obs_num, 1);
		Sig = malloc_mat(init_num_of_param, init_num_of_param);
		X   = malloc_mat(init_num_of_param, 1);

		for (int i = 0; i < 20; i++)
		{
			for (int j = 0; j < init_obs_num; j++)
			{
				S[j] = current_solution->distance_towards(&init_sat_loc[j]);
				DX0[j] = init_sat_loc[j].X - current_solution->X;
				DY0[j] = init_sat_loc[j].Y - current_solution->Y;
				DZ0[j] = init_sat_loc[j].Z - current_solution->Z;
			}

			for (int j = 0; j < init_obs_num; j++)
			{
				D->data[j][j] = 0.001;
				Z->data[j][0] = init_obs[j] - S[j];
				H->data[j][0] = -DX0[j] / S[j];
				H->data[j][1] = -DY0[j] / S[j];
				H->data[j][2] = -DZ0[j] / S[j];
			}

			for (int j = 0; j < required_init_epoch; j++)
			{
				for (int k = 0; k < satellite_amount; k++)
				{
					H->data[j * satellite_amount + k][3 + k] = lambdaL1;
					H->data[j * satellite_amount + k][3 + satellite_amount + j] = 1;
				}
			}

			LMS(Z, H, D, X, Sig, V);

			current_solution->X += X->data[0][0];
			current_solution->Y += X->data[1][0];
			current_solution->Z += X->data[2][0];
			for (int j = 0; j < required_init_epoch; j++)
			{
				T->data[j][0] += X->data[3 + satellite_amount + j][0];
			}
			for (int j = 0; j < satellite_amount; j++)
			{
				N[PRN[j]] += (X->data[3 + j][0]);
			}

			if (before.distance_towards(current_solution) <= 0.01) {

				break;
			}

			memcpy(&before, current_solution, sizeof(XYZ));
		}


		free_mat(T);
		free_mat(Z);
		free_mat(H);
		free_mat(D);
		free_mat(X);
		free_mat(Sig);
		free_mat(V);



		return true;
	}


	void fetch_available(GNSSDataSet & set, GPSTime * pre)
	{
		satellite_amount = 0;
		for (int i = 1; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			bool b1 = set.nav[i].good();
			bool b2 = set.obs[i].good(used, NUM_OF_OBS);
			if (b1 && b2)
			{
				if (set.nav[i].get_position(
					pre,
					set.obs[i].values[obs_type],
					&sat_temp
				)
					)
				{
					if (set.current_solution.X != 0)
					{
						SpaceTool::elevation_and_azimuth(&sat_temp, &set.current_solution, ea);
						if (SpaceTool::get_deg(ea[0]) < 10)
							continue;
					}

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
		Broadcast  &  nav = set.nav[index];

		double phase = obs.values[L1] * lambdaL1;
		double code  = obs.values[C1] * lambdaL1;
		
		double dtc = pre->minus(&nav.toc) - code / LIGHT_SPEED;
		double s = nav.sv_clock_bias
			+ nav.sv_clock_drift      * dtc
			+ nav.sv_clock_drift_rate * dtc;

		double r = (R_1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);

		observation[satellite_amount] =   code + s * LIGHT_SPEED; //星钟差
		observation[satellite_amount] -=         r * LIGHT_SPEED; //相对论
		observation[satellite_amount] -=   nav.tgd * LIGHT_SPEED; //群延迟

																  //地球自转改正
		double dA = we * (observation[satellite_amount] / LIGHT_SPEED - s + r);
		satellite_position[satellite_amount].X = sat_loc->X + sat_loc->Y * dA;
		satellite_position[satellite_amount].Y = sat_loc->Y - sat_loc->X * dA;
		satellite_position[satellite_amount].Z = sat_loc->Z;

		PRN[satellite_amount] = index;
		satellite_amount++;
		return true;
	}

};