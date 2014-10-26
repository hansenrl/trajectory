#pragma once

// CTrajectory command target

#include <vector>
#include "MDPoint.h"

#include <set>
using namespace std;


// this structure contains the information required to derive the lower and upper bounds
typedef struct PartitionInfo {
	float maxPerpDist;
	float minLength;
	float maxLength;
	float maxTheta;
} PartitionInfo;

class CTrajectory
{
friend class COutlierDetector;
friend class CDistanceOutlier;
public:
	CTrajectory();
	CTrajectory(int id, int nDimensions);
	virtual ~CTrajectory();
private:
	int m_trajectoryId;		// the identifier of this trajectory
	int m_nDimensions;		// the dimensionality of this trajectory
	int m_nPoints;			// the number of points constituting a trajectory 
	vector <CMDPoint> m_pointArray;	// the array of the trajectory points
	int m_nPartitionPoints;		// the number of partition points in a trajectory
	vector <CMDPoint> m_partitionPointArray;	// the array of the partition points
	vector <int> m_partitionIndexArray;		// the array of the partition indexes
	vector <PartitionInfo> m_partitionInfoArray;	// the array of the partition information
	float m_totalPartitionLength;	// the total length of all trajectory partitions
	float m_outlyingPartitionLength;	// the total length of all outlying partitions
	int m_nOutlyingPartitions;	// the number of outlying trajectory partitions
	vector <int> m_outlyingPartitionArray;	// the array of the indexes of outlying partitions
	int m_nPenWidth;		// the width of the pen for drawing a trajectory
	bool m_containOutlier;	// true if this trajectory contains an outlier; false otherwise
public:
	void SetId(int id) { m_trajectoryId = id; }
	const int GetId() const { return m_trajectoryId; }
	void AddPointToArray(CMDPoint point) { m_pointArray.push_back(point); m_nPoints++; }
	void AddPartitionPointToArray(CMDPoint point, int index) { m_partitionPointArray.push_back(point); m_partitionIndexArray.push_back(index); m_nPartitionPoints++; }
	void StorePartitionInfo(PartitionInfo info) { m_partitionInfoArray.push_back(info); }
	void SetLength(float length) { m_totalPartitionLength = length; }
	const float GetLength() const { return m_totalPartitionLength; }
	void SetOutlyingLength(float length) { m_outlyingPartitionLength = length; }
	const float GetOutlyingLength() const { return m_outlyingPartitionLength; }
	const int GetNumOutlyingPartition() const { return m_nOutlyingPartitions; }
	const vector<CMDPoint> GetPointArray() const { return m_pointArray; }
	void AddOutlyingPartition(int index) { m_outlyingPartitionArray.push_back(index); m_nOutlyingPartitions++; }
	pair<CMDPoint*,CMDPoint*> GetOutlyingPartition(int nth);
//	bool DrawTrajectory(CDC* pDC);
#ifdef __SHOW_TRAJECTORY_PARTITION__
	BOOL DrawSimplifiedTrajectory(CDC* pDC);
#endif	// __SHOW_TRAJECTORY_PARTITION__
};
