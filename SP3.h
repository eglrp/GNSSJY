#pragma once
#include "Input.h"
#include "DataStore.h"

#include "ProblemDef.h"

enum SP3TYPE {
	POSITION,
	VELOCITY,
};
class SP3OrbitFileInput : public FileInput, public PreciseOrbitInput, public SP3Input
{
	SP3TYPE type;
	char line_buffer[100];
protected:
	bool parse_header()
	{
		// get the type of this sp3 file
		fgets(line_buffer, 100, fp);

		if (line_buffer[2] == 'P') type = POSITION;
		else if (line_buffer[2] == 'V') type = VELOCITY;
		else throw FILE_FORMAT_NOT_CORRECT;

		// then jump out of the whole header
		for (int i = 0; i < 17; i++) fgets(line_buffer, 100, fp);

		// then jump out of comments
		for (int i = 0; i < 4; i++)
		{
			fgets(line_buffer, 100, fp);
			if (strncmp(line_buffer, "/*", 2) != 0)
				throw FILE_FORMAT_NOT_CORRECT;
		}

		return true;
	}
public:
	SP3OrbitFileInput(std::wstring fn)
	{
		media = InputMedia::IN_DISK;
		protocol = InputProtocol::IN_SP3;
		usage = InputUsage::IN_PRECISE_ORBIT;
		filename = fn;

		fp = _wfopen(fn.c_str(), L"r");
		if (!fp) throw FILE_NOT_FOUND;
	}

	virtual bool try_once(PreciseOrbitObserved * obs, UTC * time)
	{
		int place = ftell(fp);
		
		// GET TIME
		UTC frame;
		double sec;
		fscanf(fp, "*%d%d%d%d%d%lf\n", 
			&frame.year, &frame.month, &frame.date, &frame.hour, &frame.minute, &sec);
		frame.year -= frame.year >= 2000 ? 2000 : 1900;
		frame.sec = round(sec);

		if (!frame.equals(time)) {
			fseek(fp, place, SEEK_SET);
			return false;
		}

		// parsing parameters
		for (int i = 0; i < 32; i++)
		{
			fgets(line_buffer, 100, fp);
			sscanf(line_buffer + 4, "%lf%lf%lf%lf",
				&obs[i].value.X, &obs[i].value.Y, &obs[i].value.Z, &obs[i].clk
			);
		}

		return true;
	}
};