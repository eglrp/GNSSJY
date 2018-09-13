#pragma once
#include <opencv2/opencv.hpp>
using cv::imwrite;
using cv::Mat;
using cv::Size;
using cv::Scalar;
using cv::Point;

#include <string>


using std::wstring;
#include "DataStore.h"
#include "ProblemDef.h"

enum OutputMedia
{
	OUT_DISK,
	OUT_TCP,
	OUT_SERIAL,
	OUT_MEUNKNOWN,
};

enum OutputProtocol{
	OUT_TXT,
	OUT_IMG,
};

enum OutputUsage{
	OUT_SOLUTION,

	OUT_USUNKNOWN,
};


class Output{

protected:
	OutputMedia media;
	OutputProtocol protocol;
	OutputUsage usage;

	bool any_error;

public:
	virtual bool put_once(GNSSDataSet & set) = 0;
	virtual void end() = 0;
};

class FileOutput: public virtual Output{
protected:
	wstring filename;
	FILE * fp;

public:
	//virtual bool is_valid() = 0;
};


class SolutionOutput: public virtual Output{
protected:

public:
	//virtual void get_station_info(StationInfo * out) = 0;
	//virtual void put_once(Observation * obs, UTC * time) = 0;
};

class TXTOutput: public virtual Output{

protected:

public:

};

class IMGOutput: public virtual Output{

	protected:

	public:

};

// gw gs X Y Z To
class TXTSolutionFileOutput: public TXTOutput, public SolutionOutput, public FileOutput{
protected:


public:
	TXTSolutionFileOutput(wstring fn)
	{
		media = OutputMedia::OUT_DISK;
		protocol = OutputProtocol::OUT_TXT;
		usage = OutputUsage::OUT_SOLUTION;
		filename = fn;

		fp = _wfopen(fn.c_str(), L"w");
		if(fp == NULL) throw LACK_OF_ACCESS;
	}

	virtual bool put_once(GNSSDataSet & set)
	{
		GPSTime gps(set.obs_time);
		fprintf(fp, "%d\t%d\t%.4lf\t%.4lf\t%.4lf\t%.4lf\n", gps.week, gps.sec,
			set.current_solution.X, set.current_solution.Y, set.current_solution.Z, set.To);
		return true;
	}
	virtual void end()
	{
		fclose(fp);
	}

};

enum WindowSize {
	SIZE_100M,
	SIZE_50M,
	SIZE_10M,
	SIZE_5M,
	SIZE_1M,
	SIZE_0_5M,
	SIZE_0_05M
};

class IMGSolutionFileOutput: public IMGOutput, public SolutionOutput, public FileOutput
{
protected:
	double MAT_EDGE;
	Size   MAT_SIZE;
	double LINE_WIDTH;
	double TEXT_SIZE;
	double POINT_RADIUS;

	double CENTER_POINT_RADIUS;

#define NUM_OF_SCALE  3


	Mat paint;

	double x_bias, y_bias, scale;
	double scales[NUM_OF_SCALE];

	const Scalar ui_colors[9] = {
		Scalar(0xFA,0xF0,0xE6,0xFF),
		Scalar(0x76,0xEE,0xC6,0xFF),
		Scalar(0x8B,0x25,0x00,0xFF),
		Scalar(0x8B,0x0A,0x50,0xFF),
		Scalar(0x8B,0x00,0x8B,0xFF),
		Scalar(0x19,0x19,0x70,0xFF),
		Scalar(0x17,0x17,0x17,0xFF),
		Scalar(0x55,0x6B,0x2F,0xFF),
		Scalar(0x5C,0x5C,0x5C,0xFF)
	};

	void draw_point(XYZ & xyz, XYZ & ref)
	{
		ENU pre;
		xyz.getENULocationFrom(ref, e, pre);

		double factual_x = pre.E;
		double factual_y = pre.N;
		int paint_x = (int)((pre.E + x_bias) * scale + 0.5);
		int paint_y = (int)((-(pre.N + y_bias) * scale) + MAT_EDGE);
		Point point(paint_x, paint_y);
		circle(paint, point, (int)POINT_RADIUS, ui_colors[7], (int)LINE_WIDTH);
		if (is_first)
		{
			is_first = false;
		}
		else {
			if (draw_line) line(paint, last_point, point, ui_colors[2], (int)LINE_WIDTH);
		}
		last_point = point;
	}

	void draw_center()
	{
		double factual_x = 0;
		double factual_y = 0;
		int paint_x = (int)((x_bias) * scale + 0.5);
		int paint_y = (int)((-(y_bias) * scale) + MAT_EDGE);
		Point point(paint_x, paint_y);
		circle(paint, point, (int)CENTER_POINT_RADIUS, ui_colors[3], (int)LINE_WIDTH);
	}

	void draw_scale()
	{
		Scalar color = Scalar(0, 0, 0, 255);

		Point p2 = Point(MAT_EDGE / 10, MAT_EDGE / 10);
		Point p1 = Point(MAT_EDGE / 10, MAT_EDGE / 10 + MAT_EDGE / 100);
		line(paint, p1, p2, color, LINE_WIDTH, CV_AA);
		putText(paint, "0", p2, CV_FONT_HERSHEY_DUPLEX, TEXT_SIZE, color, LINE_WIDTH, 8, 0);

		for (int i = 0; i < NUM_OF_SCALE; i++)
		{
			double length = scales[i] * scale;
			Point pm = Point(MAT_EDGE / 10 + length, MAT_EDGE / 10 + MAT_EDGE / 100);
			Point pm2 = Point(MAT_EDGE / 10 + length, MAT_EDGE / 10);
			line(paint, pm, pm2, color, LINE_WIDTH, CV_AA);
			line(paint, pm, p1, color, LINE_WIDTH, CV_AA);
			char temp[10] = "";
			sprintf(temp, "%lg", scales[i]);
			putText(paint, temp, pm2, CV_FONT_HERSHEY_DUPLEX, TEXT_SIZE, color, LINE_WIDTH, 8, 0);
		}
	}

	virtual void set_edge(double value)
	{
		MAT_EDGE = value;
		MAT_SIZE = Size(MAT_EDGE, MAT_EDGE);
		LINE_WIDTH = MAT_EDGE / 2000;
		TEXT_SIZE = MAT_EDGE / 2000;
		POINT_RADIUS = MAT_EDGE / 3000;
		CENTER_POINT_RADIUS = 10 * POINT_RADIUS;
	}



	virtual void fetch_size(WindowSize size)
	{
		double factual_edge = 0;
		switch (size)
		{
		case SIZE_100M:
			factual_edge = 100;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 50 + (factual_edge / 20);
			y_bias = 50 + (factual_edge / 20);
			scales[2] = 30;
			scales[1] = 15;
			scales[0] = 5;
			break;
		case SIZE_50M:
			factual_edge = 50;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 25 + (factual_edge / 20);
			y_bias = 25 + (factual_edge / 20);
			scales[2] = 20;
			scales[1] = 10;
			scales[0] = 5;
			break;
		case SIZE_10M:
			factual_edge = 10;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 5 + (factual_edge / 20);
			y_bias = 5 + (factual_edge / 20);
			scales[2] = 3;
			scales[1] = 1;
			scales[0] = 0.5;
			break;
		case SIZE_5M:
			factual_edge = 5;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 2.5 + (factual_edge / 20);
			y_bias = 2.5 + (factual_edge / 20);
			scales[2] = 2;
			scales[1] = 1;
			scales[0] = 0.5;
			break;
		case SIZE_1M:
			factual_edge = 1;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 0.5 + (factual_edge / 20);
			y_bias = 0.5 + (factual_edge / 20);
			scales[2] = 0.3;
			scales[1] = 0.1;
			scales[0] = 0.05;
			break;
		case SIZE_0_5M:
			factual_edge = 0.5;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 0.25 + (factual_edge / 20);
			y_bias = 0.25 + (factual_edge / 20);
			scales[2] = 0.3;
			scales[1] = 0.1;
			scales[0] = 0.05;
			break;
		case SIZE_0_05M:
			factual_edge = 0.05;
			scale = MAT_EDGE / (factual_edge + (factual_edge / 10));
			x_bias = 0.025 + (factual_edge / 20);
			y_bias = 0.025 + (factual_edge / 20);
			scales[2] = 0.02;
			scales[1] = 0.01;
			scales[0] = 0.005;
			break;
		}
	}

	std::vector<XYZ> locs;
	XYZ * center;
	bool draw_line;
	Point last_point;
	bool is_first;
	Matrix * e;
public:
	IMGSolutionFileOutput() = default;
	IMGSolutionFileOutput(std::wstring fn, WindowSize size, XYZ * center = NULL, bool draw_line = false, double edge = 10000.0)
	{
		media = OutputMedia::OUT_DISK;
		protocol = OutputProtocol::OUT_IMG;
		usage = OutputUsage::OUT_SOLUTION;
		filename = fn;
		set_edge(edge);
		fetch_size(size);
		this->center = center;
		this->draw_line = draw_line;
		is_first = true;

		if (center)
		{
			BLH center_blh;
			center->toBLH(&center_blh);
			e = center_blh.getENUTrans();
		}
		else {
			e = NULL;
			locs.reserve(1000);
		}

		paint = Mat(MAT_SIZE, CV_8UC3, ui_colors[0]);
		draw_scale();
	}
	virtual void reserve(int length)
	{
		locs.reserve(length);
	}


	virtual bool put_once(GNSSDataSet & set)
	{
		if (!set.solution_available()) return false;
		if (center)
		{
			draw_point(set.current_solution, *center);
		}
		else {
			locs.push_back(set.current_solution);
		}
		return true;
	}
	virtual void end()
	{
		if (!center)
		{
			XYZ mean_loc = { 0,0,0 };
			int length = locs.size();
			double valid_amount = length;
			for (int i = 0; i < length; i++)
			{
				if (locs[i].X == 0)
				{
					valid_amount--;
					continue; // skip empty values
				}
				mean_loc.X += locs[i].X / length;
				mean_loc.Y += locs[i].Y / length;
				mean_loc.Z += locs[i].Z / length;
			}
			mean_loc.X *= length / valid_amount;
			mean_loc.Y *= length / valid_amount;
			mean_loc.Z *= length / valid_amount;
			BLH mean_blh;
			mean_loc.toBLH(&mean_blh);
			e = mean_blh.getENUTrans();
			for (int i = 0; i < length; i++)
			{
				draw_point(locs[i], mean_loc);
			}
			draw_center();
		}
		else {
			draw_center();
		}
		unsigned len = filename.size() * 4;
		setlocale(LC_CTYPE, "");
		char *p = new char[len];
		wcstombs(p, filename.c_str(), len);
		std::string str1(p);
		delete[] p;

		imwrite(str1, paint);
	}

};