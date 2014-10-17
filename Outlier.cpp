// Outlier.cpp : implementation file
//

//#include "stdafx.h"
#include "TraOutlier.h"
#include "Outlier.h"


// COutlier

COutlier::COutlier()
{
	m_outlierId = -1;
	m_nDimensions = 2;
	m_trajectoryId = -1;
	m_nOutlyingPartitions = 0;
	m_outlyingRatio = 0.0;
	m_nPenWidth = 3;
}

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
/*
BOOL COutlier::DrawOutlier(CDC* pDC)
{
	if (m_nDimensions > 2)
	{
        AfxMessageBox(TEXT("Unable to visualize high-dimensional data"));
        return FALSE;
	}

	CPen penOutlier;
	if (!penOutlier.CreatePen(PS_SOLID, m_nPenWidth, RGB(255,0,0)))		// red color
		return FALSE;
	CPen* pOldPen = pDC->SelectObject(&penOutlier);

	CPoint startPoint, endPoint;
	for (int i = 0; i < m_nOutlyingPartitions; i++)
	{
		startPoint.SetPoint((int)(m_outlyingPartitionArray[i].first.GetCoordinate(0)), (int)(m_outlyingPartitionArray[i].first.GetCoordinate(1)));
		endPoint.SetPoint((int)(m_outlyingPartitionArray[i].second.GetCoordinate(0)), (int)(m_outlyingPartitionArray[i].second.GetCoordinate(1)));
		pDC->MoveTo(startPoint);
		pDC->LineTo(endPoint);
	}

	pDC->SelectObject(pOldPen);
	return TRUE;
}
*/
