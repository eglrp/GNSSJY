#pragma once
#include <vector>
#include "Solver.h"
#include "Input.h"
#include "Output.h"
#include "RINEX2.h"

class Locator{
private:
	Solver * solver;
	std::vector<Input *> in;
	//std::vector

	std::vector<Output*> out;

	GNSSDataSet * dataset;


public:



	Locator() = default;
	Locator(SolverType t){
		out = std::vector<Output*>();
		in = std::vector<Input*>();
		dataset = new GNSSDataSet();

		switch(t)
		{
			case SPP:
			solver = new SimpleSolver();
				break;
			case PPP:
			// under construction
				break;
		}
	}

	void set_file_input(wstring filename, InputProtocol protocol, InputUsage usage)
	{
		//if(input != NULL) delete input;

		if(protocol == RINEX2)
		{
			if(usage == OBSERVATION)
			{
				in.push_back(new RINEX2ObservationFileInput(filename));
			}
			else if(usage == NAVIGATION)
			{
				// under construction
			}
		}
		else if(protocol == RINEX3)
		{

		}

	}
};