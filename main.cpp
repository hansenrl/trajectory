#include <iostream>
#include <fstream>
#include <string>
#include <climits>
#include "trajData.h"
using namespace std;

int main () {
    // Say HelloWorld five times
    for (int index = 0; index < 1; ++index)
      cout << "HelloWorld!" << endl;
    char input = 'i';
    cout << "To exit, press 'm' then the 'Enter' key." << endl;
    cin  >> input;
    while(input != 'm') {
        cout << "You just entered '" << input << "'. "
             << "You need to enter 'm' to exit." << endl;
        cin  >> input;
    }
    cout << "Thank you. Exiting." << endl;
    return 0;
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
        return false;
    }

	istr.getline(buffer, sizeof(buffer));	// the number of dimensions
	sscanf(buffer, "%d", &nDimensions);
	m_nDimensions = nDimensions;
	istr.getline(buffer, sizeof(buffer));	// the number of trajectories
	sscanf(buffer, "%d", &nTrajectories);
	m_nTrajectories = nTrajectories;

	m_maxNPoints = INT_MIN;					// initialize for comparison
/*
	// the trajectory Id, the number of points, the coordinate of a point ...
	for (int i = 0; i < nTrajectories; i++)
	{
		offset = 0;

		istr.getline(buffer, sizeof(buffer));	// each trajectory
		sscanf_s(buffer, "%d %d", &trajectoryId, &nPoints);
		sscanf_s(buffer, "%s %s", tmp_buf1, sizeof(tmp_buf1), tmp_buf2, sizeof(tmp_buf2));
		offset += (int)strlen(tmp_buf1) + (int)strlen(tmp_buf2) + 2;

		if (nPoints > m_maxNPoints) m_maxNPoints = nPoints;
		m_nLineSegments += (nPoints - 1);
		nTotalPoints += nPoints;

		CTrajectory* pTrajectoryItem = new CTrajectory(trajectoryId, nDimensions);
		m_trajectoryList.AddTail(pTrajectoryItem);

		for (int j = 0; j < nPoints; j++)
		{
			CMDPoint point(nDimensions);	// initialize the CMDPoint class for each point

			for (int k = 0; k < nDimensions; k++)
			{
				sscanf_s(buffer + offset, "%f", &value);
				sscanf_s(buffer + offset, "%s", tmp_buf1, sizeof(tmp_buf1));
				offset += (int)strlen(tmp_buf1) + 1;

				point.SetCoordinate(k, value);
			}

			pTrajectoryItem->AddPointToArray(point);
		}
	}

	istr.close();
	*/
	return true;
}
