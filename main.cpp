#include <iostream>
#include <fstream>
#include <string>
#include "TrajData.h"
#include "OutlierDetector.h"

using namespace std;

int main () {
	// TrajData reads in data and holds trajectory information
	TrajData data;
	data.readFile("hurricane2000_2006.tra");

	// COutlierDetector will find outliers in the provided dataset
	COutlierDetector outlierDetector(&data);
	outlierDetector.PartitionTrajectory();
	outlierDetector.DetectOutlier();
	cout << "Size of trajectory outlierList: " << data.m_outlierList.size() << "\n";
	cout << "Finished partitioning and finding outliers!\n";

	// Output plot of trajectories and outliers
	data.OutputTrajectoryPlot("sampleOut.eps");

    return 0;
}
