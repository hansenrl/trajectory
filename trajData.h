/*
 * trajData.h
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
// DEBUG ...
#ifdef __VISUALIZE_DEBUG_INFO__
	float m_lineSegmentDensity;
	int m_nCloseTrajectories;
	int m_nCloseLineSegments;
	CPoint m_currStartPoint;
	CPoint m_currEndPoint;
	CArray<CPoint,CPoint> m_startPointArray;
	CArray<CPoint,CPoint> m_endPointArray;
#endif
// ... DEBUG

// Implementation
public:
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

};


#endif /* TRAJDATA_H_ */
