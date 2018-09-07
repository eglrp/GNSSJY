#pragma once
#include <string>
using std::wstring;

enum OutputMedia
{
	OUT_DISK,
	OUT_TCP,
	OUT_SERIAL,
	OUT_NTRIP,
	OUT_MEUNKNOWN,
};

enum OutputProtocol{
	OUT_TXT,
	OUT_BMP,
	OUT_RTCM3,
	OUT_NOVATEL3,
	OUT_PRUNKNOWN,
};

enum OutputUsage{
	OUT_SOLUTION,
	OUT_NAVIGATION,

	OUT_PRECISE_ORBIT,
	OUT_EARTH_ROTATION,
	OUT_PRECISE_CLOCK,
	OUT_USUNKNOWN,
};


class Output{

protected:
	OutputMedia media;
	OutputProtocol protocol;
	OutputUsage usage;

	bool any_error;

public:
	virtual wstring description() = 0;

};

class FileOutput: public virtual Output{
protected:
	wstring filename;
	FILE * fp;

public:
	virtual bool is_valid() = 0;
};


class SolutionOutput: public virtual Input{
protected:

public:
	//virtual void get_station_info(StationInfo * out) = 0;
	virtual void get_once(Observation * obs, UTC * time) = 0;
};