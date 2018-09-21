

#include <stdlib.h>
#include <stdio.h>

#include "DataStore.h"
#include "RINEX2.h"
#include "Output.h"
#include "JTime.h"
#include "DIYSolver.h"

//int main()
//{
//	merge_rinex2(L"C:\\Users\\80575\\Desktop\\hr");
//
//}
int main()
{
	GNSSDataSet dataset;
	RINEX2ObservationFileInput inputo(L"daej2070.SMT");
	RINEX2NavigationFileInput  inputn(L"daej2070.18N");
	//RINEXIonosphereFileInput   inputi(L"whug1960.18i", &dataset.tec);
	dataset.sta = inputo.get_sta();
	
	IMGSolutionFileOutput outputt(
		L"daej2070.sln.bmp", SIZE_50M, 
		dataset.sta->approx_position.X == 0 ? NULL : &dataset.sta->approx_position, 
		//NULL,
		false,
		10000
	);
	outputt.reserve(10000);
	TXTSolutionFileOutput output2(L"daej2070.sln.txt");
	
	SimplePPPSolver solver;
	//SimpleSmoothedSolver solver(/*inputo.get_interval()*/1);
	//SimpleKinematicSolver solver(inputo.get_interval());

	//DIYSolver solverd;
	//for (int i = 0; i < 3.40E4; i++) {
	//	inputo.get_once(dataset.obs, &dataset.obs_time);
	//	inputn.try_once(dataset.nav, &dataset.obs_time);
	//}
	//SimpleAmbiguitySolver solver(dataset);
	int epoch_counter = 0;
	
	try {

		//while (true)
		for(int i = 0; i < 2.51E4; i++)
		{
			inputo.get_once( dataset.obs, &dataset.obs_time);
			inputn.try_once( dataset.nav, &dataset.obs_time);

			if (solver.execute(dataset)) {

				outputt.put_once(dataset);
				output2.put_once(dataset);

				epoch_counter++;
			}
		}
	}
	catch (MyError error_code)
	{
		printf("%d\n", error_code);
	}
	outputt.end();
	output2.end();
	_fcloseall();
	system("pause");
	return 0;
}