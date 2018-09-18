#pragma once
#include "Space.h"

// using model Hopefield and NMF
class SimpleTroposphereModel {
public:
	void set_location(XYZ & loc)
	{
		loc.toBLH(&location);
	}
	void set_date(int doy) {
		t = cos(2 * PI * (doy - 28) / 365.25);
	}
	void set_parameters(XYZ & loc, int doy)
	{
		loc.toBLH(&location);
		t = cos(2 * PI * (doy - 28) / 365.25);
	}

	SimpleTroposphereModel() = default;

	SimpleTroposphereModel(XYZ & loc, int doy)
	{
		loc.toBLH(&location);
		t = cos(2 * PI * (doy - 28) / 365.25);
	}

	double get_dry_std(double elev)
	{
		return get_dry_m(elev) * get_dry_ztd();
	}

	double get_overall_std(double elev)
	{
		return get_dry_std(elev) + get_wet_std(elev);
	}

	double get_wet_std(double elev)
	{
		return get_wet_m(elev) * get_wet_ztd();
	}

	double get_dry_ztd()
	{
		double elev = PI / 2;
		double t0, p0, e0, h0;
		double t, p;
		double dend, elev2, hd, rkd;
		double hgt = location.H;

		if (fabs(hgt)>30000.0)   return 0.0;

		t0 = 20 + 273.16;
		p0 = 1013.0;
		e0 = 0.5 * exp(-37.2465 + 0.213166 * t0 - 0.000256908 * t0 * t0);
		h0 = 0;
		t = t0 - 0.0068 * (hgt - h0);
		p = p0 * pow(1.0 - 0.0068 / t0 * (hgt - h0), 5);

		elev2 = elev * elev * Deg * Deg;
		dend = sqrt(elev2 + 6.25) / Deg;

		hd = 148.72 * t0 - 488.3552;
		rkd = 1.552e-5 * p / t * (hd - hgt);
		return  (rkd / sin(dend));
	}

	double get_wet_m(double elev)
	{
		double lat_degree = fabs(SpaceTool::get_deg(location.B));
		if (lat_degree > 75)lat_degree = 75;
		else if (lat_degree < 15)lat_degree = 15;
		int lat_index = (int)floor(lat_degree / lat_step) - 1;

		double a, b, c;
		a = wet_abc_ave[0][lat_index] +
			(wet_abc_ave[0][lat_index + 1] - wet_abc_ave[0][lat_index]) *
			(lat_degree - lat_grid[lat_index]) / lat_step;

		b = wet_abc_ave[1][lat_index] +
			(wet_abc_ave[1][lat_index + 1] - wet_abc_ave[1][lat_index]) *
			(lat_degree - lat_grid[lat_index]) / lat_step;

		c = wet_abc_ave[2][lat_index] +
			(wet_abc_ave[2][lat_index + 1] - wet_abc_ave[2][lat_index]) *
			(lat_degree - lat_grid[lat_index]) / lat_step;

		double sinE = sin(elev);
		return (1.0 / (1.0 + a / (1.0 + b / (1.0 + c)))) / (1.0 / (sinE + a / (sinE + b / (sinE + c))));

	}

	double get_wet_ztd()
	{
		double elev = PI / 2;
		double t0, p0, e0, h0;
		double t, e;
		double elev2, denw, hw, rkw;
		double hgt = location.H;

		if (fabs(hgt)>30000.0)   return 0.0;

		t0 = 20 + 273.16;
		p0 = 1013.0;
		e0 = 0.5*exp(-37.2465 + 0.213166*t0 - 0.000256908*t0*t0);
		h0 = 0;
		hw = 11000.0;
		t = t0 - 0.0068*(hgt - h0);
		e = e0*pow((1 - 0.0068 / t0*(hgt - h0)), 2.0)*pow((1.0 - (hgt - h0) / hw), 4.0);
		elev2 = elev*elev * Deg * Deg;
		denw = sqrt(elev2 + 2.25) / Deg;

		rkw = 7.46512e-2*(e / t / t)*(hw - hgt);
		return (rkw / sin(denw));
	}

	double get_dry_m(double elev)
	{
		double lat_degree = fabs(SpaceTool::get_deg(location.B));
		int lat_index = (int)floor(lat_degree / lat_step) - 1;

		double a, b, c;
		switch (lat_index)
		{
		case -1:
			a = dry_abc_ave[0][0] + dry_abc_ave[0][0] * t;
			b = dry_abc_ave[1][0] + dry_abc_ave[1][0] * t;
			c = dry_abc_ave[2][0] + dry_abc_ave[2][0] * t;
			break;
		case 4:
			a = dry_abc_ave[0][4] + dry_abc_ave[0][4] * t;
			b = dry_abc_ave[1][4] + dry_abc_ave[1][4] * t;
			c = dry_abc_ave[2][4] + dry_abc_ave[2][4] * t;
			break;
		default:
			a = dry_abc_ave[0][lat_index] +
				(dry_abc_ave[0][lat_index + 1] - dry_abc_ave[0][lat_index]) *
				(lat_degree - lat_grid[lat_index]) / lat_step;
			a += dry_abc_amp[0][lat_index] +
				(dry_abc_amp[0][lat_index + 1] - dry_abc_amp[0][lat_index]) *
				(lat_degree - lat_grid[lat_index]) / lat_step * t;

			b = dry_abc_ave[1][lat_index] +
				(dry_abc_ave[1][lat_index + 1] - dry_abc_ave[1][lat_index]) *
				(lat_degree - lat_grid[lat_index]) / lat_step;
			b += dry_abc_amp[1][lat_index] +
				(dry_abc_amp[1][lat_index + 1] - dry_abc_amp[1][lat_index]) *
				(lat_degree - lat_grid[lat_index]) / lat_step * t;

			c = dry_abc_ave[2][lat_index] +
				(dry_abc_ave[2][lat_index + 1] - dry_abc_ave[2][lat_index]) *
				(lat_degree - lat_grid[lat_index]) / lat_step;
			c += dry_abc_amp[2][lat_index] +
				(dry_abc_amp[2][lat_index + 1] - dry_abc_amp[2][lat_index]) *
				(lat_degree - lat_grid[lat_index]) / lat_step * t;
			break;

		}
		double sinE = sin(elev);
		return (1.0 / (1.0 + a / (1.0 + b / (1 + c)))) / (1.0 / (sinE + a / (sinE + b / (sinE + c)))) +
			(1.0 / sinE - (1.0 / (1.0 + abc_ht[0] / (1.0 + abc_ht[1] / (1.0 + abc_ht[2])))) / (1.0 / (sinE + abc_ht[0] / (sinE + abc_ht[1] / (sinE + abc_ht[2]))))) * location.H / 1000;
	}
protected:
	BLH location;
	double t;


	const double abc_ht[3]{ 2.53E-5, 5.49E-3,1.14E-3 };
	const int lat_step = 15;
	const int lat_grid[5]{ 15, 30,45, 60,75 };

	const double dry_abc_ave[3][5]{
		{1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
		{2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
		{62.620505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3}
	};

	const double dry_abc_amp[3][5]{
		{   0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
		{   0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
		{   0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5}
	};

	const double wet_abc_ave[3][5]{
		{5.8021879E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
		{1.4275268E-3, 1.5138625E-3, 1.4572572E-3, 1.5007428E-3, 1.7599082E-3},
		{4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736039E-2}
	};
private:

};