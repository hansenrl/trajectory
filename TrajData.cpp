/*
 * TrajData.cpp
 *
 *      Author: Ross H
 */

#include <iostream>
#include <fstream>
#include <string>
#include <climits>
#include <cstring>
#include <vector>
#include <tuple>
#include <set>
#include "TrajData.h"
#include "MDPoint.h"
#include "Trajectory.h"
#include "OutlierDetector.h"
#include "Param.h"
#include "gnuplot-iostream.h"

/**
 * \brief Default constructor. Uses the fraction parameter p and the distance parameter D specified in params.h
 */
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

/**
 * \brief Output an eps image of the trajectories and outliers.
 *
 * Makes a plot of the trajectories with gnuplot and highlights outlying trajectories and partitions.
 * Uses default coloring and line widths: outlier trajectories colored red, line widths 2,
 * and outlier partitions in a line width of 6.
 *
 * The output file is specified with filePath, which should be an .eps file.
 *
 * @param filePath [in] The desired path and filename of the eps file to output
 */
void TrajData::OutputTrajectoryPlot(string filePath){
	OutputTrajectoryPlot(filePath, "post eps color", "red", 2, "red", 6);
}

/**
 * \brief Output a png image of the trajectories and outliers.
 *
 * Makes a plot of the trajectories with gnuplot and highlights outlying trajectories and partitions.
 * Uses default coloring and line widths: outlier trajectories colored red, line widths 2,
 * and outlier partitions in a line width of 6.
 *
 * The output file is specified with filePath, which should be an .png file.
 *
 * @param filePath [in] The desired path and filename of the png file to output
 */
void TrajData::OutputTrajectoryPlotPNG(string filePath){
	OutputTrajectoryPlot(filePath, "pngcairo enhanced", "red", 2, "red", 6);
}

/**
 * \brief Output an eps image of the trajectories and outliers.
 *
 * Makes a plot of the trajectories with gnuplot and highlights outlying trajectories and partitions.
 * The output file is specified with filePath, which should be an eps file.
 *
 * @param filePath [in] The desired path and filename of the eps file to output
 * @param termSettings [in] the terminal settings passed to gnuplot to specify the output type and style. For example, for eps files specify "post eps", for png output specify "pngcairo", etc. Options can be specified afterward, such as "post eps color".
 * @param oTrajColor [in] The color for outlying trajectories
 * @param oTrajWidth [in] The line width for outlying trajectories
 * @param oPartColor [in] The color for outlying partitions
 * @param oPartWidth [in] The line width for outlying partitions
 */
void TrajData::OutputTrajectoryPlot(string filePath, string termSettings, string oTrajColor, int oTrajWidth, string oPartColor, int oPartWidth){
	set<int> outlier_trajectories;
	for (COutlier* outlier_p : m_outlierList)
	{
		outlier_trajectories.insert((*outlier_p).GetTrajectoryId());
	}

	// Setup terminal
	Gnuplot gp;
	gp << "set term " << termSettings << "\n";
	gp << "set output '" << filePath << "'\n";
	gp << "unset key\n";

	// Draw normal trajectories and outlier trajectories
	if(outlier_trajectories.find(m_trajectoryList[0]->GetId()) == outlier_trajectories.end())
	{
		gp << "plot '-' using 1:2 with lines title '0' linetype -1 lw 1" ;
	} else {
		gp << ", '' using 1:2 with lines title '0' linetype 1 lc rgb '" << oTrajColor << "' lw " << oTrajWidth;
	}
	for(int i = 1; i < m_trajectoryList.size(); i++){
		if(outlier_trajectories.find(m_trajectoryList[i]->GetId()) == outlier_trajectories.end())
		{
			gp << ", '' using 1:2 with lines title '0' linetype -1 lw 1";
		} else {
			gp << ", '' using 1:2 with lines title '0' linetype 1 lc rgb '" << oTrajColor << "' lw " << oTrajWidth;
		}
	}

	// Find number of outlying partitions and setup plotting for them
	int totalOutlyingPartitions = 0;
	for( COutlier* outlier : m_outlierList){
		totalOutlyingPartitions += outlier->GetNOutlyingPartitions();
	}
	for(int i = 0; i < totalOutlyingPartitions; i++){
		gp << ", '' using 1:2 with lines title '0' linetype 1 lc rgb '" << oPartColor << "' lw " << oPartWidth;
	}
	gp << "\n";

	// Draw outlying partitions
	for( CTrajectory* traj_p : m_trajectoryList){
		vector<tuple<float, float> > pts;
		CTrajectory traj = *traj_p;
		vector<CMDPoint> points = traj.GetPointArray();
		for ( CMDPoint point : points){
			pts.push_back(make_tuple(point.GetCoordinate(0), point.GetCoordinate(1)));
		}

		gp.send1d(pts);
	}
	int i = 0;
	for( COutlier* outlier : m_outlierList){
		auto partitions = outlier->GetOutlyingPartitionArray();
		for( auto partition : partitions){
			vector<tuple<float, float> > pts;
			pts.push_back(make_tuple(partition.first.GetCoordinate(0),partition.first.GetCoordinate(1)));
			pts.push_back(make_tuple(partition.second.GetCoordinate(0),partition.second.GetCoordinate(1)));
			gp.send1d(pts);
			i++;
			if(i == 2) {
				//break;
			}
		}
	}
}

/**
 * \brief Read trajectories from an input .traj file
 *
 *
 * @param[in] filePath The path to the input .traj file
 * @return True if read is successful
 */
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

	istr.close();

	return true;
}
