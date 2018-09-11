#pragma once
// This file is to define basic input types and their interfaces.
// 


#include <string>
#include "JTime.h"
#include "DataStore.h"

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
	IN_SP3,
	IN_NOVATEL3,
	IN_PRUNKNOWN,
};

enum InputUsage{
	IN_OBSERVATION,
	IN_NAVIGATION,
	IN_IONOSPHERE,
	IN_DIFFERENTIAL,

	IN_PRECISE_ORBIT,
	IN_EARTH_ROTATION,
	IN_PRECISE_CLOCK,
	IN_USUNKNOWN,
};

struct RinexFileHeader {
	// RINEX VERSION / TYPE
	double rinex_version;
	char file_type[20];
	char satellite_system[20];

	// PGM / RUN BY / DATE
	char name_of_pgm[21];
	char name_of_agency[21];
	char data_of_create[21];
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
};
class PreciseOrbitInput : public virtual Input {
protected:
public:
	virtual bool try_once(PreciseOrbitObserved * obs, UTC * time) = 0;
};
class SP3Input : public virtual Input {
protected:
public:
	
};
class ObservationInput: public virtual Input{
protected:

public:
	virtual bool get_once(Observation * obs, UTC * time) = 0;
};

class NavigationInput: public virtual Input{
protected:

public:
	virtual bool try_once(Broadcast * nav, UTC * time) = 0;
};

class IonosphereInput: public virtual Input{
protected:

public:
	virtual bool try_once(TECMap * map, UTC * time) = 0;
};

class TCPInput : public virtual Input {
protected:
public:
};

class DifferentialInput : public virtual Input {
protected:
public:
};

class RTCM3Input : public virtual Input {
protected:
public:
};

class RINEX2Input: public virtual Input{
protected:
	char line_buffer[100];
	virtual bool parse_header() = 0;
	virtual int parse_header_line(const char * content, const char * title) = 0;
public:

};