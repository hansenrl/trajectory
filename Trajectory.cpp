// Trajectory.cpp : implementation file
//

//#include "stdafx.h"
#include "TraOutlier.h"
#include "Trajectory.h"


// CTrajectory

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
/*
BOOL CTrajectory::DrawTrajectory(CDC* pDC)
{
	if (m_nDimensions > 2)
	{
        AfxMessageBox(TEXT("Unable to visualize high-dimensional data"));
        return FALSE;
	}

	CPen penTrajectory1, penTrajectory2;
	if (!penTrajectory1.CreatePen(PS_SOLID, m_nPenWidth, RGB(0,255,0)))	// green color
		return FALSE;
	if (!penTrajectory2.CreatePen(PS_SOLID, m_nPenWidth, RGB(255,0,0)))	// hot pink: RGB(255,105,180)
		return FALSE;
	CPen* pOldPen = pDC->SelectObject(&penTrajectory1);

	CPoint point;
	point.SetPoint((int)(m_pointArray[0].GetCoordinate(0)), (int)(m_pointArray[0].GetCoordinate(1)));
	pDC->MoveTo(point);
	for (int i = 1; i < m_pointArray.GetSize(); i++)
	{
		if (!m_containOutlier) pDC->SelectObject(&penTrajectory1);
		else pDC->SelectObject(&penTrajectory2);
		point.SetPoint((int)(m_pointArray[i].GetCoordinate(0)), (int)(m_pointArray[i].GetCoordinate(1)));
		pDC->LineTo(point);
	}

	pDC->SelectObject(pOldPen);
	return TRUE;
}
*/
#ifdef __SHOW_TRAJECTORY_PARTITION__
BOOL CTrajectory::DrawSimplifiedTrajectory(CDC* pDC)
{
	if (m_nDimensions > 2)
	{
        AfxMessageBox(TEXT("Unable to visualize high-dimensional data"));
        return FALSE;
	}

	// if partitioning has not been done yet
	if (m_nPartitionPoints == 0) return TRUE;

	CPen penTrajectory1, penTrajectory2;
	if (!penTrajectory1.CreatePen(PS_DOT, m_nPenWidth, RGB(160,32,240)))	// purple
		return FALSE;
	if (!penTrajectory2.CreatePen(PS_DOT, m_nPenWidth, RGB(0,0,255)))	// blue color
		return FALSE;
	CPen* pOldPen = pDC->SelectObject(&penTrajectory1);

	CPoint point;
	point.SetPoint((int)(m_partitionPointArray[0].GetCoordinate(0)), (int)(m_partitionPointArray[0].GetCoordinate(1)));
	pDC->MoveTo(point);
	for (int i = 1; i < m_partitionPointArray.GetSize(); i++)
	{
		if (i % 2 == 0) pDC->SelectObject(&penTrajectory1);
		else pDC->SelectObject(&penTrajectory2);
		point.SetPoint((int)(m_partitionPointArray[i].GetCoordinate(0)), (int)(m_partitionPointArray[i].GetCoordinate(1)));
		pDC->LineTo(point);
	}

	pDC->SelectObject(pOldPen);
	return TRUE;
}
#endif	// __SHOW_TRAJECTORY_PARTITION__
