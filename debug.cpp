

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
	RINEX2ObservationFileInput inputo(L"bjfs2500.18o");
	RINEX2NavigationFileInput  inputn(L"bjfs2500.18n");
	//RINEXIonosphereFileInput   inputi(L"whug1960.18i", &dataset.tec);
	dataset.sta = inputo.get_sta();
	
	IMGSolutionFileOutput outputt(
		L"bjfs2500.sln.bmp", SIZE_100M, 
		dataset.sta->approx_position.X == 0 ? NULL : &dataset.sta->approx_position, 
		true,
		10000
	);
	outputt.reserve(10000);

	TXTSolutionFileOutput output2(L"bjfs2500.sln.txt");
	

	SimpleSmoothedSolver solver(inputo.get_interval());
	//SimpleKinematicSolver solver(inputo.get_interval());
	//SimpleSolver solver;

	//GPSTime pre = GPSTime(dataset.obs_time);
	int epoch_counter = 0;
	
	try {
		while (true)
		{
			inputo.get_once( dataset.obs, &dataset.obs_time);
			inputn.try_once( dataset.nav, &dataset.obs_time);
			//inputi.try_once(&dataset.tec, &dataset.obs_time);
			solver.execute(  dataset);
			outputt.put_once(dataset);
			output2.put_once(dataset);
			epoch_counter++;
		}
	}
	catch (MyError error_code)
	{
		printf("%d\n", error_code);
	}
	outputt.end();
	output2.end();

	system("pause");
}