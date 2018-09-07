#pragma once
#include <string>
using std::wstring;

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
	OUT_BMP,
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