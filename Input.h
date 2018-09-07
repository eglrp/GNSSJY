#pragma once
// This file is to define basic input types and their interfaces.
// 


#include <string>

#include "DataStore.h"
#include "JTime.h"
using std::wstring;

enum InputMedia
{
	IN_DISK,
	IN_TCP,
	IN_SERIAL,
	IN_NTRIP,
	IN_MEUNKNOWN,
};

enum InputProtocol{
	IN_RINEX2,
	IN_RINEX3,
	IN_RTCM3,
	IN_NOVATEL3,
	IN_PRUNKNOWN,
};

enum InputUsage{
	IN_OBSERVATION,
	IN_NAVIGATION,

	IN_PRECISE_ORBIT,
	IN_EARTH_ROTATION,
	IN_PRECISE_CLOCK,
	IN_USUNKNOWN,
};


class Input{

protected:
	InputMedia media;
	InputProtocol protocol;
	InputUsage usage;

	bool any_error;

public:
	//virtual wstring description() = 0;

};

class FileInput: public virtual Input{
protected:
	wstring filename;
	FILE * fp;

public:
	virtual bool is_valid() = 0;
};


class ObservationInput: public virtual Input{
protected:

public:
	virtual bool get_once(Observation * obs, UTC * time) = 0;
};

class NavigationInput: public virtual Input{
protected:

public:
	
};

class RINEX2Input: public virtual Input{
protected:
	virtual bool parse_header() = 0;
	virtual int parse_header_line(const char * content, const char * title) = 0;
public:

};