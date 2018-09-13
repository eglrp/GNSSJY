#pragma once
#include "Solver.h"

class DIYSolver : public SimpleSolver {

	// 此类用来进行不同形式的GNSS 解算对结果所造成的影响，不属于正常程序的一部分，后期会移除。
	// 第一版本的内容包括：
	// 1. 无电离层组合的结果。涉及到的输出自己解决并不依靠Output接口。
	// 2. 宽窄巷法解算浮点模糊度的解以及固定收敛的情况。
	// 3. 模糊度改正后载波相位观测的解
	// 4. 基于高次差法的周跳检测
	// 4. 多历元共同求解的测相伪距绝对定位法。
public:
	DIYSolver(): SimpleSolver() {
		f12 = FREQ1 * FREQ1;
		f22 = FREQ2 * FREQ2;
		f12_f22 = FREQ1 * FREQ1 - FREQ2 * FREQ2;

		double l1G = 1.57542;
		double l2G = 1.22760;

		//double t1 = f12 / f12_f22;
		//double t2 = f22 / f12_f22;
		f12df12_f22 = (l1G * l1G) / (l1G * l1G - l2G * l2G);
		f22df12_f22 = (l2G * l2G) / (l1G * l1G - l2G * l2G);

		f1mf1_f2 = FREQ1 / (FREQ1 - FREQ2);
		f2mf1_f2 = FREQ2 / (FREQ1 - FREQ2);

		f1mf1pf2 = FREQ1 / (FREQ1 + FREQ2);
		f2mf1pf2 = FREQ2 / (FREQ1 + FREQ2);

		lambda1 = LIGHT_SPEED / FREQ1;
		lambda2 = LIGHT_SPEED / FREQ2;

		if ((solution2 = fopen("SOLUTION2.txt", "w")) == NULL) throw LACK_OF_ACCESS;
		if ((solution1 = fopen("SOLUTION1.txt", "w")) == NULL) throw LACK_OF_ACCESS;
		if ((amb1      = fopen("AMB1.txt",      "w")) == NULL) throw LACK_OF_ACCESS;
		if ((amb2      = fopen("AMB2.txt",      "w")) == NULL) throw LACK_OF_ACCESS;

		period_counter = 0;
		is_first = true;
	}

	virtual bool execute(GNSSDataSet & set)
	{
		GPSTime * pre = &GPSTime(set.obs_time);
		// firstly, calculate the positions of all available satellites
		fetch_available(set, pre);
		if (satellite_amount <= 3) return false;


		get_static_solution(&set.current_solution, set.To);

		fprintf(solution1, "%lf\t%lf\t%lf\t%lf\n", set.current_solution.X, set.current_solution.Y, set.current_solution.Z, set.To);
		period_counter++;
		for (int i = 0; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			fprintf(amb1, "%lf\t", ambiguity_l1[i]);
			fprintf(amb2, "%lf\t", ambiguity_l2[i]);
		}
		fprintf(amb1, "\n");
		fprintf(amb2, "\n");
		is_first = false;
		return true;
	}

protected:
	TYPE_OF_RINEX_OBS used[4] = { L1, C1, L2, P2 };


	// 文件接口
	FILE * solution1;
	FILE * solution2;
	FILE * amb1;
	FILE * amb2;

	// 基本变量
	double ambiguity_l1[GNSS_SATELLITE_AMOUNT];
	double ambiguity_l2[GNSS_SATELLITE_AMOUNT];
	//double obs[4][GNSS_SATELLITE_AMOUNT];
	


	void fetch_available(GNSSDataSet & set, GPSTime * pre)
	{
		satellite_amount = 0;
		for (int i = 1; i < GNSS_SATELLITE_AMOUNT; i++)
		{
			ambiguity_l1[i] = 0;
			ambiguity_l2[i] = 0;
			bool b1 = set.nav[i].good();
			bool b2 = set.obs[i].good(used, 4);
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

					// make the typical corrections done
					if (!pre_process(pre, set, i, &sat_temp))
					{
						continue;
					}
				}
			}
		}
	}

	// for IF
	double f12;
	double f22;
	double f12_f22;
	double f12df12_f22;
	double f22df12_f22;

	// for WL
	double f1mf1_f2;
	double f2mf1_f2;

	// for nl
	double f1mf1pf2;
	double f2mf1pf2;

	double lambda1, lambda2;


	// smoothing
	double last_P1[GNSS_SATELLITE_AMOUNT];
	double last_P2[GNSS_SATELLITE_AMOUNT];
	double last_L1[GNSS_SATELLITE_AMOUNT];
	double last_L2[GNSS_SATELLITE_AMOUNT];

	int period_counter;
	bool is_first;

	virtual bool pre_process(GPSTime * pre, GNSSDataSet & set, int index, XYZ * sat_loc)
	{
		Observation & obs = set.obs[index];
		Broadcast  & nav = set.nav[index];
		if(is_first)
		{ 
			double p = 0.5 * obs.values[C1] + 0.5 * f22df12_f22 * obs.values[P2];
			double dtc = pre->minus(&nav.toc) - p / LIGHT_SPEED;
			double s = nav.sv_clock_bias
				+ nav.sv_clock_drift      * dtc
				+ nav.sv_clock_drift_rate * dtc;

			double r = (R_1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);

			observation[satellite_amount] = p + s * LIGHT_SPEED;//星钟差
			observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
			observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟

																   //地球自转改正
			double dA = we * (observation[satellite_amount] / LIGHT_SPEED - s + r);
			satellite_position[satellite_amount].X = sat_loc->X + sat_loc->Y * dA;
			satellite_position[satellite_amount].Y = sat_loc->Y - sat_loc->X * dA;
			satellite_position[satellite_amount].Z = sat_loc->Z;
		}
		else {
			double delta = lambda1 * (obs.values[L1] - last_L1[index]);
			double f1 = obs.values[C1] / period_counter;
			double f2 = last_P1[index] + delta;
			double f3 = (period_counter - 1.0) / period_counter * f2;
			obs.values[C1] = f1 + f3;

			delta = lambda2 * (obs.values[L2] - last_L2[index]);
			f1 = obs.values[P2] / period_counter;
			f2 = last_P2[index] + delta;
			f3 = (period_counter - 1.0) / period_counter * f2;
			obs.values[P2] = f1 + f3;
			

			double dtc = pre->minus(&nav.toc) - obs.values[C1] / LIGHT_SPEED;
			double s = nav.sv_clock_bias
				+ nav.sv_clock_drift      * dtc
				+ nav.sv_clock_drift_rate * dtc;

			double r = (R_1 * nav.eccentricity * sin(nav.Ek) * nav.sqrt_a);

			observation[satellite_amount] = obs.values[C1] + s * LIGHT_SPEED;//星钟差
			observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
			observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟

			obs.values[C1] = observation[satellite_amount];

			observation[satellite_amount] = obs.values[P2] + s * LIGHT_SPEED;//星钟差
			observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
			observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟

			obs.values[P2] = observation[satellite_amount];

			observation[satellite_amount] = obs.values[L1] * lambda1 + s * LIGHT_SPEED;//星钟差
			observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
			observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟

			obs.values[L1] = observation[satellite_amount] / lambda1;

			observation[satellite_amount] = obs.values[L2] * lambda2 + s * LIGHT_SPEED;//星钟差
			observation[satellite_amount] -= r * LIGHT_SPEED; //相对论
			observation[satellite_amount] -= nav.tgd * LIGHT_SPEED;//群延迟

			obs.values[L2] = observation[satellite_amount] / lambda2;

			//地球自转改正
			double dA = we * (observation[satellite_amount] / LIGHT_SPEED - s + r);
			satellite_position[satellite_amount].X = sat_loc->X + sat_loc->Y * dA;
			satellite_position[satellite_amount].Y = sat_loc->Y - sat_loc->X * dA;
			satellite_position[satellite_amount].Z = sat_loc->Z;


			// 无电离层线性组合
			double Pif = f12df12_f22 * obs.values[C1] - f22df12_f22 * obs.values[P2];

			double Lif = f12df12_f22 * obs.values[L1] * lambda1 - f22df12_f22 * obs.values[L2] * lambda2;
			double Pwl =
				f1mf1_f2 * obs.values[C1] - f2mf1_f2 * obs.values[P2];
			double Lwl =
				f1mf1_f2 * obs.values[L1] * lambda1 - f2mf1_f2 * obs.values[L2] * lambda2;
			double Pnl =
				f1mf1pf2 * obs.values[C1] + f2mf1pf2 * obs.values[P2];
			double Lnl =
				f1mf1pf2 * obs.values[L1] * lambda1 + f2mf1pf2 * obs.values[L2] * lambda2;

			ambiguity_l2[index] = (FREQ1 + FREQ2) / (2 * LIGHT_SPEED) * ((Lnl - Pnl) - (Lwl - Pwl));
			ambiguity_l1[index] = (FREQ2 * 1.0 / FREQ1) * ambiguity_l2[index] + f12_f22 / (LIGHT_SPEED * FREQ1) * (Lif - Pif);

			observation[satellite_amount] = Pif;
		}

		last_L1[index] = obs.values[L1];
		last_P1[index] = obs.values[C1];
		last_L2[index] = obs.values[L2];
		last_P2[index] = obs.values[P2];




		satellite_amount++;
		return true;
	}

};