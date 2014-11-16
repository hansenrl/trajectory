#pragma once

// COutlier command target

#include "MDPoint.h"
#include "Trajectory.h"
#include <vector>

#include <set>
using namespace std;


typedef pair<CMDPoint,CMDPoint> LineSegment;

/**
 * \brief A trajectory outlier
 *
 * Holds information useful for working with a trajectory that is an outlier - such as the outlying partitions
 * that are part of this trajectory
 */
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
		///< Setter for the outlier ID
	const int GetId() const { return m_outlierId; }
		///< Getter for the outlier ID
	const int GetTrajectoryId() const { return m_trajectoryId; }
		///< Getter for the trajectory ID
	const int GetNOutlyingPartitions() const { return m_nOutlyingPartitions; }
		///< Getter for the number of outlying partitions in the outlying trajectory
	const vector<LineSegment> GetOutlyingPartitionArray() const { return m_outlyingPartitionArray; }
		///< Getter for the array of outlying partitions
	const float GetOutlyingRatio() const { return m_outlyingRatio; }
		///< Getter for the ratio of outlying partition length to total length
	void SetupInfo(CTrajectory* pTrajectory);
};
