#include <iostream>
#include <fstream>
#include <string>
#include <climits>
#include <cstring>
#include "trajData.h"
#include "MDPoint.h"
#include "Trajectory.h"
#include "OutlierDetector.h"
#include "Param.h"

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
