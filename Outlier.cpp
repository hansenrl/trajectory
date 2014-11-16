// Outlier.cpp : implementation file
//

#include "Param.h"
#include "Outlier.h"


// COutlier

/**
 * \brief Constructor for 2-dimensional data, sets the outlier and trajectory IDs to -1
 */
COutlier::COutlier()
{
	m_outlierId = -1;
	m_nDimensions = 2;
	m_trajectoryId = -1;
	m_nOutlyingPartitions = 0;
	m_outlyingRatio = 0.0;
	m_nPenWidth = 3;
}

/**
 * \brief Constructor
 *
 * @param [in] id outlier ID
 * @param [in] trajectoryId trajectory ID
 * @param [in] nDimensions the number of dimensions in the data
 */
COutlier::COutlier(int id, int trajectoryId, int nDimensions)
{
	m_outlierId = id;
	m_nDimensions = nDimensions;
	m_trajectoryId = trajectoryId;
	m_nOutlyingPartitions = 0;
	m_outlyingRatio = 0.0;
	m_nPenWidth = 3;
}

COutlier::~COutlier()
{
	m_nOutlyingPartitions = 0;
}


// COutlier member functions
/**
 * \brief Pulls the relevant information from the trajectory into this outlier object
 *
 * @param [in] pTrajectory the trajectory object for the outlier
 */
void COutlier::SetupInfo(CTrajectory* pTrajectory)
{
	// setup the number of outlying trajectory partitions
	m_nOutlyingPartitions = pTrajectory->GetNumOutlyingPartition();

	// setup the points of outlying trajectory partitions
	for (int i = 0; i < m_nOutlyingPartitions; i++)
	{
		// get each outlying trajectory partition
		pair <CMDPoint*, CMDPoint*> aPartition = pTrajectory->GetOutlyingPartition(i);

		CMDPoint startPoint, endPoint;
		for (int j = 0; j < m_nDimensions; j++)
		{
			startPoint.SetCoordinate(j, aPartition.first->GetCoordinate(j));
			endPoint.SetCoordinate(j, aPartition.second->GetCoordinate(j));
		}
		m_outlyingPartitionArray.push_back(LineSegment(startPoint, endPoint));
	}

	// setup the ratio of outlying trajectory partitions
	m_outlyingRatio = pTrajectory->GetOutlyingLength() / pTrajectory->GetLength();

	return;
}
