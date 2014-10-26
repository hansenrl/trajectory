#pragma once

// COutlier command target

#include "MDPoint.h"
#include "Trajectory.h"
#include <vector>

#include <set>
using namespace std;


typedef pair<CMDPoint,CMDPoint> LineSegment;

class COutlier
{
friend class COutlierDetector;
public:
	COutlier();
	COutlier(int id, int trajectoryId, int nDimensions);
	virtual ~COutlier();
private:
	int m_outlierId;		// the identifier of this outlier
	int m_nDimensions;		// the dimensionality of this outlier
	int m_trajectoryId;		// the identifier of the trajectory that this outlier belongs to
	int m_nOutlyingPartitions;	// the number of outlying trajectory partitions in this outlier
	vector <LineSegment> m_outlyingPartitionArray;	// the array of outlying trajectory partitions
	float m_outlyingRatio;	// the ratio of the outlying length to the entire length
	int m_nPenWidth;		// the width of the pen for drawing a cluster
public:
	void SetId(int id) { m_outlierId = id; }
	const int GetId() const { return m_outlierId; }
	const int GetTrajectoryId() const { return m_trajectoryId; }
	const float GetOutlyingRatio() const { return m_outlyingRatio; }
	void SetPenWidth(int penWidth) { m_nPenWidth = penWidth; }
	void SetupInfo(CTrajectory* pTrajectory);
	//BOOL DrawOutlier(CDC* pDC);
};
