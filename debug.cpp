

#include <stdlib.h>
#include <stdio.h>

#include "DataStore.h"
#include "RINEX2.h"
#include "Output.h"
#include "Solver.h"
#include "JTime.h"


int main()
{
	GNSSDataSet dataset;
	RINEX2ObservationFileInput inputo(L"20160620.16O");
	RINEX2NavigationFileInput inputn(L"20160620.16N");

	
	TXTSolutionFileOutput outputt(L"20160620.sln.txt");

	SimpleSmoothedSolver solver(inputo.get_interval());
	//SimpleKinematicSolver solver(inputo.get_interval());
	//SimpleSolver solver;
	dataset.sta = inputo.get_sta();

	//GPSTime pre = GPSTime(dataset.obs_time);
	//FILE * fp = fopen("debug_result.txt", "w");
	int epoch_counter = 0;
	try {
		while (true)
		{
			//if (epoch_counter == 25)
			//{
			//	int i = 0;
			//}
			inputo.get_once(dataset.obs, &dataset.obs_time);
			inputn.try_once(dataset.nav, &dataset.obs_time);
			solver.execute(dataset);
			outputt.put_once(dataset);
			epoch_counter++;
			//system("pause");
		}
	}
	catch (error error_code)
	{
		printf("%d\n", error_code);
	}
	outputt.end();

	system("pause");
}