// Trajectory.cpp : implementation file
//

#include "TraOutlier.h"
#include "Trajectory.h"

// CTrajectory

/**
 * \brief Default constructor
 *
 * Sets the trajectory ID to -1 and the number of dimensions to 2.
 */
CTrajectory::CTrajectory()
{
	m_trajectoryId = -1;
	m_nDimensions = 2;
	m_nPoints = 0;
	m_nPartitionPoints = 0;
	m_totalPartitionLength = 0.0;
	m_outlyingPartitionLength = 0.0;
	m_nOutlyingPartitions = 0;
	m_nPenWidth = 1;	// thin width
	m_containOutlier = false;
}

/**
 * \brief Constructor with id and dimension parameters
 *
 * @param [in] id the trajectory ID
 * @param [in] nDimensions the number of dimensions for the points in the trajectory (typically 2)
 */
CTrajectory::CTrajectory(int id, int nDimensions)
{
	m_trajectoryId = id;
	m_nDimensions = nDimensions;
	m_nPoints = 0;
	m_nPartitionPoints = 0;
	m_totalPartitionLength = 0.0;
	m_outlyingPartitionLength = 0.0;
	m_nOutlyingPartitions = 0;
	m_nPenWidth = 1;	// thin width
	m_containOutlier = false;
}

CTrajectory::~CTrajectory()
{
	m_nPoints = 0;
	m_containOutlier = false;
}


// CTrajectory member functions

/**
 * \brief Get the nth outlying partition in this trajectory
 *
 * @param [in] nth Which outlying partition of the trajectory to return
 * @return A pair of CMDPoint pointers which define the start and end points of the partition segment
 */
pair<CMDPoint*, CMDPoint*> CTrajectory::GetOutlyingPartition(int nth)
{
	int index = m_outlyingPartitionArray[nth];
	pair<CMDPoint*, CMDPoint*> lineSegment;

#if defined(__USE_CONVENTIONAL_PARTITONING__) && !defined(__PARTITION_PRUNING_OPTIMIZATION__)
	lineSegment.first  = &(m_partitionPointArray[index]);
	lineSegment.second = &(m_partitionPointArray[index + 1]);
#elif defined(__USE_NO_PARTITIONING__) || defined(__PARTITION_PRUNING_OPTIMIZATION__)
	lineSegment.first  = &(m_pointArray[index]);
	lineSegment.second = &(m_pointArray[index + 1]);
#endif

	return lineSegment;
}
