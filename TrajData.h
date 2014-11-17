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
	std::string m_inputFilePath; ///< The path of the input file
	int m_nDimensions; ///< The dimensionality of the dataset (typically 2)
	int m_nTrajectories; ///< The number of trajectories in the dataset
	int m_nOutliers; ///< The number of outliers found
	int m_nOutlyingPartitions; ///< The outlying partitions found

	std::vector<CTrajectory*> m_trajectoryList; ///< The list of trajectories
	std::list<COutlier*> m_outlierList; ///< The list of outliers
	float m_paramFraction; ///< The parameter p (fraction of trajectories that must be not close to be an outlier)
	float m_paramDistance; ///< The distance parameter D
	int m_nLineSegments; ///< The number of line segments in the original data
	int m_nTrajectoryPartitions; ///< The number of trajectory partitions

	bool readFile(string);
	void OutputTrajectoryPlot(string);
	void OutputTrajectoryPlotPNG(string filePath);
	void OutputTrajectoryPlot(string filePath, string termSettings, string oTrajColor, int oTrajWidth, string oPartColor, int oPartWidth);

private:
	int m_maxNPoints; ///< The maximum number of points to read in
};


#endif /* TRAJDATA_H_ */
