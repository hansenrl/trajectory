#include <iostream>
#include <fstream>
#include <string>
#include <climits>
#include <cstring>
#include <vector>
#include <tuple>
#include <set>
#include "trajData.h"
#include "MDPoint.h"
#include "Trajectory.h"
#include "OutlierDetector.h"
#include "Param.h"
#include "gnuplot-iostream.h"

using namespace std;

int main () {
    // Say HelloWorld five times
    /*for (int index = 0; index < 1; ++index)
      cout << "HelloWorld!" << endl;
    char input = 'i';
    cout << "To exit, press 'm' then the 'Enter' key." << endl;
    cin  >> input;
    while(input != 'm') {
        cout << "You just entered '" << input << "'. "
             << "You need to enter 'm' to exit." << endl;
        cin  >> input;
    }
    cout << "Thank you. Exiting." << endl;*/
	TrajData data;
	//data.readFile("/home/ross/traod_expr/real_data/Hurricane/hurricane1950_2006.tra");
	data.readFile("/home/ross/traod_expr/real_data/Hurricane/hurricane2000_2006.tra");

	COutlierDetector outlierDetector(&data);
	outlierDetector.PartitionTrajectory();
	outlierDetector.DetectOutlier();
	cout << "Size of outlierList: " << data.m_outlierList.size() << "\n";
	cout << "Finished!\n";
	data.OutputTrajectoryPlot("/home/ross/test.eps");

	/*
	gp << "set term post eps color\n";
	gp << "set output '/home/ross/test.eps'\n";
	vector<tuple<double, double, double, double> > pts_A;
	for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
		double theta = alpha*2.0*3.14159;
		pts_A.push_back(make_tuple(
			 cos(theta),
			 sin(theta),
			-cos(theta)*0.1,
			-sin(theta)*0.1
		));
	}
	gp << "plot '-' with vectors title 'pts_A', '-' with vectors title 'pts_B'\n";
	gp.send1d(pts_A);
	*/
    return 0;
}

TrajData::TrajData(){
	m_nTrajectories = 0;
	m_nOutliers = 0;
	m_nOutlyingPartitions = 0;
	m_nLineSegments = 0;
	m_nTrajectoryPartitions = 0;

	m_paramFraction = g_FRACTION_PARAMETER;
	m_paramDistance = g_DISTANCE_PARAMETER;
}

TrajData::~TrajData(){

}

void TrajData::OutputTrajectoryPlot(string filePath){
	set<int> outlier_trajectories;
	for (COutlier* outlier_p : m_outlierList)
	{
		outlier_trajectories.insert((*outlier_p).GetTrajectoryId());
	}

	Gnuplot gp;
	gp << "set term post eps color\n";
	gp << "set output '" << filePath << "'\n";
	gp << "unset key\n";

	if(outlier_trajectories.find(m_trajectoryList[0]->GetId()) == outlier_trajectories.end())
	{
		gp << "plot '-' using 1:2 with lines title '0' linetype -1";
	} else {
		gp << "plot '-' using 1:2 with lines title '0' linetype 1";
	}
	for(int i = 1; i < m_trajectoryList.size(); i++){
		if(outlier_trajectories.find(m_trajectoryList[i]->GetId()) == outlier_trajectories.end())
		{
			gp << ", '' using 1:2 with lines title '0' linetype -1";
		} else {
			gp << ", '' using 1:2 with lines title '0' linetype 1";
		}
	}
	gp << "\n";

	for( CTrajectory* traj_p : m_trajectoryList){
		vector<tuple<float, float> > pts;
		CTrajectory traj = *traj_p;
		vector<CMDPoint> points = traj.GetPointArray();
		for ( CMDPoint point : points){
			pts.push_back(make_tuple(point.GetCoordinate(0), point.GetCoordinate(1)));
		}

		gp.send1d(pts);
	}
}

bool TrajData::readFile(string filePath)
{
	int nDimensions = 2;		// default dimension = 2
	int nTrajectories = 0;
	int nTotalPoints = 0;
	int trajectoryId;
	int nPoints;
	float value;
	char buffer[1024000];
	char tmp_buf1[1024], tmp_buf2[1024];
	int offset = 0;

    ifstream istr(filePath.c_str());


    if(!istr)
    {
    	cout << "Could not read file " << filePath << "\n";
        return false;
    }

	istr.getline(buffer, sizeof(buffer));	// the number of dimensions
	sscanf(buffer, "%d", &nDimensions);
	m_nDimensions = nDimensions;
	istr.getline(buffer, sizeof(buffer));	// the number of trajectories
	sscanf(buffer, "%d", &nTrajectories);
	m_nTrajectories = nTrajectories;

	m_maxNPoints = INT_MIN;					// initialize for comparison

	// the trajectory Id, the number of points, the coordinate of a point ...
	for (int i = 0; i < nTrajectories; i++)
	{
		offset = 0;

		istr.getline(buffer, sizeof(buffer));	// each trajectory
		sscanf(buffer, "%d %d", &trajectoryId, &nPoints);
		sscanf(buffer, "%s %s", tmp_buf1, tmp_buf2);
		offset += (int)strlen(tmp_buf1) + (int)strlen(tmp_buf2) + 2;

		if (nPoints > m_maxNPoints) m_maxNPoints = nPoints;
		m_nLineSegments += (nPoints - 1);
		nTotalPoints += nPoints;

		CTrajectory* pTrajectoryItem = new CTrajectory(trajectoryId, nDimensions);
		m_trajectoryList.push_back(pTrajectoryItem);

		for (int j = 0; j < nPoints; j++)
		{
			CMDPoint point(nDimensions);	// initialize the CMDPoint class for each point

			for (int k = 0; k < nDimensions; k++)
			{
				sscanf(buffer + offset, "%f", &value);
				sscanf(buffer + offset, "%s", tmp_buf1); // see how long the float is to advance the scan
				offset += (int)strlen(tmp_buf1) + 1;

				point.SetCoordinate(k, value);
			}

			pTrajectoryItem->AddPointToArray(point);
		}
	}
	cout << "Size of list: " << m_trajectoryList.size() << "\n";
	istr.close();

	return true;
}
