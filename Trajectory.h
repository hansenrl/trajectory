#pragma once

// CTrajectory command target

#include <vector>
#include "MDPoint.h"

#include <set>
using namespace std;


// this structure contains the information required to derive the lower and upper bounds
/**
 * \brief Information required to derive the lower and upper bounds for optimized partition pruning
 */
typedef struct PartitionInfo {
	float maxPerpDist;
	float minLength;
	float maxLength;
	float maxTheta;
} PartitionInfo;

/**
 * \brief The main storage for information about a trajectory
 *
 * Represents a trajectory in the dataset, including its partitioning.
 */
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
	void SetId(int id) { m_trajectoryId = id; } ///< Set the trajectory id of this trajectory
	const int GetId() const { return m_trajectoryId; } ///< Return the trajectory id of this trajectory
	void AddPointToArray(CMDPoint point) { m_pointArray.push_back(point); m_nPoints++; } ///< Add a CMDPoint point to the array of trajectory points
	void AddPartitionPointToArray(CMDPoint point, int index) {
		m_partitionPointArray.push_back(point);
		m_partitionIndexArray.push_back(index);
		m_nPartitionPoints++;
	} ///< Add a point to the array of partition points
	void StorePartitionInfo(PartitionInfo info) { m_partitionInfoArray.push_back(info); }
		///< Add the PartitionInfo object info to the array of partition information
	void SetLength(float length) { m_totalPartitionLength = length; }
		///< Setter for total partition length
	const float GetLength() const { return m_totalPartitionLength; }
		///< Getter for total partition length
	void SetOutlyingLength(float length) { m_outlyingPartitionLength = length; }
		///< Setter for outlying partition length
	const float GetOutlyingLength() const { return m_outlyingPartitionLength; }
		///< Getter for outlying partition length
	const int GetNumOutlyingPartition() const { return m_nOutlyingPartitions; }
		///< Getter for number of outlying partitions
	const vector<CMDPoint> GetPointArray() const { return m_pointArray; }
		///< Getter for trajectory point array
	const vector <CMDPoint> GetPartitionPointArray() const { return m_partitionPointArray; }
		///< Getter for partition point array
	void AddOutlyingPartition(int index) { m_outlyingPartitionArray.push_back(index); m_nOutlyingPartitions++; }
		///< Add an outlying partition to the array of outlying partitions

	pair<CMDPoint*,CMDPoint*> GetOutlyingPartition(int nth);

};
