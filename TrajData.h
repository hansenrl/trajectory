/*
 * TrajData.h
 *
 *  Created on: Oct 16, 2014
 *      Author: ross
 */

#ifndef TRAJDATA_H_
#define TRAJDATA_H_

#include <string>
#include <list>
#include "Outlier.h"
#include "Trajectory.h"

/**
 * \brief The main class, and handler for all of the trajectory data
 *
 * The main purpose of this class is to hold the trajectory data. Contains functions to read in trajectory data and output
 * images. After outlier detection is run this class also holds information on the outlying trajectories and outlying partitions.
 */
class TrajData
{
public:
	TrajData();
	~TrajData();
	std::string m_inputFilePath;
	int m_nDimensions;
	int m_nTrajectories;
	int m_nOutliers;
	int m_nOutlyingPartitions;
	int m_maxNPoints;
	std::vector<CTrajectory*> m_trajectoryList;
	std::list<COutlier*> m_outlierList;
	float m_paramFraction;
	float m_paramDistance;
	int m_nLineSegments;
	int m_nTrajectoryPartitions;

	bool readFile(string);
	void OutputTrajectoryPlot(string);
};


#endif /* TRAJDATA_H_ */
