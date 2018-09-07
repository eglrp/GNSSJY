

#include <stdlib.h>
#include <stdio.h>

#include "DataStore.h"
#include "RINEX2.h"
#include "Solver.h"
#include "JTime.h"


int main()
{
	GNSSDataSet dataset;
	RINEX2ObservationFileInput inputo(L"bjfs2210.08o");
	RINEX2NavigationFileInput inputn(L"bjfs2210.08n");

	SimpleSolver solver;
	dataset.sta = inputo.get_sta();

	//GPSTime pre = GPSTime(dataset.obs_time);
	FILE * fp = fopen("debug_result.txt", "w");
	int epoch_counter = 0;
	try {
		while (true)
		{
			if (epoch_counter == 25)
			{
				int i = 0;
			}
			inputo.get_once(dataset.obs, &dataset.obs_time);
			inputn.try_once(dataset.nav, &dataset.obs_time);
			solver.execute(dataset);
			fprintf(fp, "%.6lf\t%.6lf\t%.6lf\t%.6lf\n", dataset.current_solution.X, dataset.current_solution.Y, dataset.current_solution.Z, dataset.To);
			epoch_counter++;
		}
	}
	catch (error error_code)
	{
		printf("%d\n", error_code);
	}
	fclose(fp);
	system("pause");
}