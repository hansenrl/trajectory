#include <iostream>
#include <fstream>
#include <string>
#include "TrajData.h"
#include "OutlierDetector.h"

using namespace std;

int main () {
	TrajData data;
	//data.readFile("/home/ross/traod_expr/real_data/Hurricane/hurricane1950_2006.tra");
	data.readFile("/home/ross/traod_expr/real_data/Hurricane/hurricane2000_2006.tra");
	//data.readFile("/home/ross/afrl_data.tra");

	COutlierDetector outlierDetector(&data);
	// set fraction distance?
	outlierDetector.PartitionTrajectory();
	outlierDetector.DetectOutlier();
	cout << "Size of outlierList: " << data.m_outlierList.size() << "\n";
	cout << "Finished!\n";
	data.OutputTrajectoryPlot("/home/ross/test.eps");

    return 0;
}
